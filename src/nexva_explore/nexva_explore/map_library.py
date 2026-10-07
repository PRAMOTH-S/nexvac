"""
The saved maps, by name.

`map_registry` already records the map that was saved LAST, per source, in
`map.md`. That is what a cleaning run needs when nobody says otherwise, and it
stays exactly as it is. What it cannot answer is "which maps exist?", because
it only ever holds one entry per source - and picking a map by name from the
terminal or the dashboard needs the whole list.

The maps are already stored per name on disk:

    ~/nexva_maps/<source>/<name>.pgm      the picture
    ~/nexva_maps/<source>/<name>.yaml     what map_server loads
    ~/nexva_maps/<source>/<name>.posegraph  slam_toolbox's own belief
    ~/nexva_maps/<source>/<name>.data      its companion
    ~/nexva_maps/<source>/<name>.as_mapped.*  the pristine copy

so listing them is reading the directory. No new format, no second store, no
migration: this module is a reader over what the autosaver already writes.

There is one more place a map can come from: `nexva_navigation/maps`, the
maps the robot was already navigating on before the autosaver existed
(sep23map1, sep23map2). They are .yaml + .pgm only - no pose graph - so they
can be served but never resumed. They are listed alongside the autosaved
maps with `origin: 'nexva_navigation'` so the dashboard can say where a map
came from, and an autosaved map wins any name collision.

SOURCES ARE KEPT APART. `hardware` is the real robot and `sim` is Gazebo, and
they never share a directory. Cleaning a simulator map on the real robot would
send it sweeping a room that is not there, so nothing here will hand back a
map from the other side by accident.
"""

import os
import re

from nexva_explore import map_registry

DEFAULT_ROOT = map_registry.DEFAULT_MAPS_ROOT

# Both halves of what slam_toolbox needs to resume, and what map_server needs
# to serve. A map missing either half is listed but not startable, and saying
# which is missing is more use than hiding it.
SERVE_PARTS = ('.yaml', '.pgm')
RESUME_PARTS = ('.posegraph', '.data')

# The pristine copy the cleaning run keeps before it starts folding live scans
# into the map. It is a backup, not a map in its own right, so it must never
# appear in the list as though it were one.
BACKUP_MARKER = '.as_mapped'

# A map name becomes a filename and a path. Anything outside this set could
# walk out of the maps directory, so it is refused rather than sanitised into
# something the user did not ask for - silently saving "kitchen" as "kitchen_"
# is how you end up with two maps and no idea which is which.
NAME_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9 _-]{0,63}$')

# Where a listed map came from. The autosaver wrote it, or it was already in
# nexva_navigation/maps before the autosaver existed.
ORIGIN_AUTOSAVED = 'autosaved'
ORIGIN_NAVIGATION = 'nexva_navigation'


def maps_root(root=None):
    """Return where the maps live, overridable for tests."""
    return map_registry.maps_root(root)


def source_dir(source, root=None):
    """Return the directory holding one side's maps."""
    return os.path.join(maps_root(root), source)


def valid_name(name):
    """
    Check a map name is safe to use as a filename.

    Returns `(cleaned, problem)`. `problem` is None when the name is fine.

    Refused rather than rewritten: a name quietly changed is a map the
    operator cannot find again.
    """
    if name is None:
        return None, 'no name given'

    cleaned = str(name).strip()

    if not cleaned:
        return None, 'the map needs a name'

    if cleaned.startswith('.'):
        return None, 'a map name cannot start with a dot'

    if BACKUP_MARKER in cleaned:
        return None, f'"{BACKUP_MARKER}" is reserved for the pristine backup'

    if not NAME_PATTERN.match(cleaned):
        return None, (
            'use letters, numbers, spaces, dashes and underscores only '
            '(no slashes, no "..", 64 characters max)')

    return cleaned, None


def map_paths(name, source, root=None):
    """Every file that belongs to one map, whether or not it exists."""
    base = os.path.join(source_dir(source, root), name)

    return {
        'base': base,
        'yaml': base + '.yaml',
        'pgm': base + '.pgm',
        'posegraph': base + '.posegraph',
        'data': base + '.data',
        'backup_pgm': base + BACKUP_MARKER + '.pgm',
        'backup_yaml': base + BACKUP_MARKER + '.yaml',
    }


def describe(name, source, root=None):
    """
    One map, and what can actually be done with it.

    `can_serve` means map_server can load it - enough to clean against.
    `can_resume` means slam_toolbox can carry on from its own pose graph,
    which is the better path and the one a cleaning run prefers.
    """
    paths = map_paths(name, source, root)

    can_serve = all(os.path.isfile(paths[part.lstrip('.')])
                    for part in SERVE_PARTS)
    can_resume = all(os.path.isfile(paths[part.lstrip('.')])
                     for part in RESUME_PARTS)

    try:
        saved_at = os.path.getmtime(paths['yaml'])
    except OSError:
        saved_at = 0.0

    try:
        size = os.path.getsize(paths['pgm'])
    except OSError:
        size = 0

    missing = [part for part in SERVE_PARTS
               if not os.path.isfile(paths[part.lstrip('.')])]

    return {
        'name': name,
        'source': source,
        'origin': ORIGIN_AUTOSAVED,
        'yaml': paths['yaml'],
        'pgm': paths['pgm'],
        'can_serve': can_serve,
        'can_resume': can_resume,
        'missing': missing,
        'saved_at': saved_at,
        'bytes': size,
    }


def describe_navigation_map(entry, source):
    """
    One of nexva_navigation's maps, in the same shape `describe` returns.

    Servable by construction - the registry only lists a .yaml that has its
    .pgm - and never resumable, because there is no pose graph for them.
    """
    try:
        saved_at = os.path.getmtime(entry['yaml'])
    except OSError:
        saved_at = 0.0

    try:
        size = os.path.getsize(entry['pgm'])
    except OSError:
        size = 0

    return {
        'name': entry['name'],
        'source': source,
        'origin': ORIGIN_NAVIGATION,
        'yaml': entry['yaml'],
        'pgm': entry['pgm'],
        'can_serve': True,
        'can_resume': False,
        'missing': [],
        'saved_at': saved_at,
        'bytes': size,
    }


def list_maps(source, root=None):
    """
    Every saved map for one source, newest first.

    Built from the `.yaml` files, because that is the one map_server actually
    loads - a stray `.pgm` with no `.yaml` is not a map anyone can use, and
    listing it would only offer a choice that fails later.

    The nexva_navigation maps are appended after the autosaved ones. On a
    name collision the autosaved map wins: it is the one the robot made
    itself, and it may carry a pose graph the older copy never had.
    """
    directory = source_dir(source, root)

    try:
        entries = os.listdir(directory)
    except OSError:
        entries = []

    names = []

    for entry in sorted(entries):
        if not entry.endswith('.yaml'):
            continue

        name = entry[:-len('.yaml')]

        if name.endswith(BACKUP_MARKER):
            # The pristine copy of another map, not a map of its own.
            continue

        names.append(name)

    found = [describe(name, source, root) for name in names]

    taken = {item['name'] for item in found}

    for entry in map_registry.nexva_navigation_maps():
        if entry['name'] in taken:
            continue

        found.append(describe_navigation_map(entry, source))

    found.sort(key=lambda item: item['saved_at'], reverse=True)

    return found


def find(name, source, root=None):
    """One named map, or None if there is no such map for this source."""
    cleaned, problem = valid_name(name)

    if problem:
        return None

    described = describe(cleaned, source, root)

    if described['can_serve']:
        return described

    # Not autosaved under that name; it may be one the robot already had.
    for entry in map_registry.nexva_navigation_maps():
        if entry['name'] == cleaned:
            return describe_navigation_map(entry, source)

    return None


def exists(name, source, root=None):
    """Whether anything is already saved under this name."""
    cleaned, problem = valid_name(name)

    if problem:
        return False

    paths = map_paths(cleaned, source, root)

    return any(os.path.exists(paths[key])
               for key in ('yaml', 'pgm', 'posegraph', 'data'))
