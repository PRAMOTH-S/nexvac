"""
Deleting a saved map, as safely as a delete can be.

`nexva_explore.map_library` can list and find maps but not remove them, and it
belongs to another package, so the delete lives here. It follows the same rule
`map_library.find()` does: the user's string is only ever COMPARED against the
listing, never joined onto a path. The files removed are rebuilt from the
listing's own entry.

Refused, never worked around:
  - a name that is not in `list_maps()`           (nothing to look up)
  - a name `valid_name()` would not accept         (traversal, dots, slashes)
  - origin 'nexva_navigation'                      (bundled in the source tree)
  - the map a running mission is using             (when /robot_mode says so)
  - an entry whose files are not inside the source's own maps directory

Only the map's own files go: .pgm .yaml .posegraph .data and the
.as_mapped.{pgm,yaml} backups. Never a directory, never anything else.
"""

import os

from nexva_explore import map_library, map_registry

SOURCE = 'hardware'
IDLE_MODES = ('', 'idle', 'stop', 'stopped', 'done')


class MapDeleteError(Exception):
    """A refusal. The message is shown to the operator as-is."""


def _in_use(robot_mode, name):
    """True/False when /robot_mode says; None when we cannot tell."""
    if not isinstance(robot_mode, dict) or not robot_mode.get('mode'):
        return None
    if robot_mode.get('map') != name:
        return False
    return str(robot_mode['mode']).lower() not in IDLE_MODES


def _resolve(name, source, root):
    cleaned, problem = map_library.valid_name(name)
    if problem:
        raise MapDeleteError('refused: %s' % problem)
    # Compare against the listing; the user string is never used as a path.
    for entry in map_library.list_maps(source, root):
        if entry['name'] == cleaned:
            return entry
    raise MapDeleteError('no saved map named "%s"' % cleaned)


def _fix_registry(name, source, root, entry):
    """Stop map.md pointing at the map that was just deleted."""
    path = os.path.join(map_registry.maps_root(root), map_registry.DEFAULT_FILENAME)
    entries = map_registry.read_all(path)
    mine = entries.get(source)
    if not mine or (mine.get('map_name') != name
                    and mine.get('map_file') != entry['yaml']):
        return 'registry untouched'

    remaining = [m for m in map_library.list_maps(source, root)
                 if m['origin'] == map_library.ORIGIN_AUTOSAVED
                 and m['name'] != name and m['can_serve']]
    others = {k: v for k, v in entries.items() if k != source}

    if remaining:
        # saved_at/latest of the repointed entry become "now": the writer has
        # no way to preserve them.
        newest = remaining[0]
        map_registry.write_registry(path, newest['name'], newest['yaml'],
                                    newest['pgm'], source)
        return 'registry now points at "%s"' % newest['name']

    os.remove(path)
    if others:
        # The writer cannot drop one source, so start the file afresh from
        # the others (oldest first, so the newest ends on top).
        for k, v in sorted(others.items(),
                           key=lambda kv: int(kv[1].get('saved_unix') or 0)):
            map_registry.write_registry(path, v.get('map_name', ''),
                                        v.get('map_file', ''),
                                        v.get('map_image', ''), k)
        return 'registry entry removed'
    return 'registry removed (no maps left)'


def delete_map(name, source=SOURCE, root=None, robot_mode=None):
    """Delete one saved map. Returns {ok, name, deleted, registry, ...}.

    Raises MapDeleteError for every refusal.
    """
    entry = _resolve(name, source, root)
    name = entry['name']

    if entry['origin'] != map_library.ORIGIN_AUTOSAVED:
        raise MapDeleteError(
            'refused: "%s" is a bundled map (%s) in the source tree, not '
            'saved user data' % (name, entry['origin']))

    if _in_use(robot_mode, name):
        raise MapDeleteError(
            'refused: "%s" is in use by the running %s mission - stop it first'
            % (name, robot_mode.get('mode')))

    base_dir = os.path.realpath(map_library.source_dir(source, root))
    paths = map_library.map_paths(name, source, root)
    deleted, skipped = [], []
    for key in ('yaml', 'pgm', 'posegraph', 'data', 'backup_pgm',
                'backup_yaml'):
        path = paths[key]
        if not os.path.lexists(path):
            continue
        # Defence in depth: must sit directly in the maps directory, and not
        # be a real directory (a symlink is unlinked, never followed).
        if (os.path.realpath(os.path.dirname(path)) != base_dir
                or (os.path.isdir(path) and not os.path.islink(path))):
            skipped.append(os.path.basename(path))
            continue
        os.remove(path)
        deleted.append(os.path.basename(path))

    if not deleted:
        raise MapDeleteError('refused: nothing to delete for "%s"' % name)

    return {'ok': True, 'name': name, 'deleted': deleted, 'skipped': skipped,
            'registry': _fix_registry(name, source, root, entry),
            'mission_state_known': _in_use(robot_mode, name) is not None}
