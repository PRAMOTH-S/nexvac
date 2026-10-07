"""
Read and write map.md, the record of the saved maps.

map.md is the single place that says which maps exist, where they live and
when they were written. The autosaver rewrites it on every save and the
cleaning launch reads it, so nothing has to be told a map path on the command
line.

Maps are kept apart by **source**. The real robot writes `hardware`, the
simulator writes `sim`, and each has its own entry and its own folder. Saving
one never touches the other, so a hardware run cannot overwrite a map Gazebo
built and cleaning on the robot reuses the map the robot itself made. That is
the whole reason this file has sections rather than one set of fields.

It is deliberately plain markdown: readable on its own, and parseable with one
regex. Both sides go through this module so the format cannot drift apart.

Maps live under `~/nexva_maps/<source>/`, and map.md lives beside them at
`~/nexva_maps/map.md` - NOT in the workspace. The workspace is rsynced between
the laptop and the Pi, and a registry that travelled with it would point at
maps that are on the other machine.
"""

import os
import re
import time

DEFAULT_FILENAME = 'map.md'

# Where a map came from. The real robot is the default now; `sim` stays a
# legal value so a Gazebo map can be brought over deliberately. Any other
# string works and simply gets its own entry.
DEFAULT_SOURCE = 'hardware'

# Where the maps live. NEXVA_MAPS overrides it, for tests or a second disk.
DEFAULT_MAPS_ROOT = os.path.join(os.path.expanduser('~'), 'nexva_maps')

_FIELD = re.compile(r'^\s*-\s*\*\*(?P<key>[a-z_]+):\*\*\s*(?P<value>.*?)\s*$')
_SECTION = re.compile(r'^##\s+(?P<source>\S+)\s*$')

_HEADER = """# Nexva saved maps

Written automatically whenever a map is saved. Do not edit by hand, it is
rewritten on the next save.

Maps are kept apart by where they came from. `hardware` is the real robot,
`sim` is Gazebo. Saving one never touches the other, and each run reuses the
map from its own source.
"""


def maps_root(root=None):
    """Return the folder every source's maps sit under."""
    return root or os.environ.get('NEXVA_MAPS') or DEFAULT_MAPS_ROOT


def registry_path(explicit=None):
    """
    Work out where map.md lives.

    Beside the maps, under ~/nexva_maps, so the registry and the files it
    points at are always on the same machine. NEXVA_MAP_REGISTRY overrides
    the whole path; NEXVA_MAPS moves the maps root and the registry with it.
    """
    if explicit:
        return explicit

    override = os.environ.get('NEXVA_MAP_REGISTRY')
    if override:
        return override

    return os.path.join(maps_root(), DEFAULT_FILENAME)


def nexva_navigation_maps():
    """
    The maps the robot already navigates on, from nexva_navigation/maps.

    These were made before the autosaver existed, so map.md has never heard
    of them - but the clean mission must still be able to pick them. Each is
    returned as {name, yaml, pgm}, newest first. The share directory is
    found through ament when the workspace is sourced, and through the
    source tree next to this package when it is not; if neither turns up
    the list is simply empty.
    """
    directory = None

    try:
        from ament_index_python.packages import get_package_share_directory
        directory = os.path.join(
            get_package_share_directory('nexva_navigation'), 'maps')
    except Exception:                                       # noqa: BLE001
        directory = None

    if not directory or not os.path.isdir(directory):
        # Unsourced shell: walk up from this file until a workspace root with
        # src/nexva_navigation/maps appears. Works from both the source tree
        # and the install tree, since both sit directly under the workspace.
        here = os.path.dirname(os.path.abspath(__file__))
        directory = None

        while True:
            candidate = os.path.join(here, 'src', 'nexva_navigation', 'maps')
            if os.path.isdir(candidate):
                directory = candidate
                break
            parent = os.path.dirname(here)
            if parent == here:
                break
            here = parent

    if not directory:
        return []

    try:
        entries = os.listdir(directory)
    except OSError:
        return []

    found = []

    for entry in sorted(entries):
        if not entry.endswith('.yaml'):
            continue

        name = entry[:-len('.yaml')]
        yaml_path = os.path.join(directory, entry)
        pgm_path = os.path.join(directory, name + '.pgm')

        if not os.path.isfile(pgm_path):
            # A .yaml on its own is not a map map_server can serve.
            continue

        found.append({'name': name, 'yaml': yaml_path, 'pgm': pgm_path})

    def stamp(item):
        try:
            return os.path.getmtime(item['yaml'])
        except OSError:
            return 0.0

    found.sort(key=stamp, reverse=True)

    return found


def read_all(path=None):
    """
    Return every recorded map as {source: fields}.

    A map.md in the older single-map format - flat fields, no sections - is
    read as one entry under the default source, so an existing file keeps
    working until the next save rewrites it.
    """
    path = registry_path(path)

    try:
        with open(path, encoding='ascii') as handle:
            lines = handle.readlines()
    except OSError:
        return {}

    entries = {}
    loose = {}
    current = None

    for line in lines:
        section = _SECTION.match(line)

        if section:
            current = section.group('source')
            entries.setdefault(current, {})
            continue

        field = _FIELD.match(line)

        if not field:
            continue

        if current is None:
            loose[field.group('key')] = field.group('value')
        else:
            entries[current][field.group('key')] = field.group('value')

    if not entries and loose.get('map_file'):
        loose.setdefault('source', DEFAULT_SOURCE)
        entries[DEFAULT_SOURCE] = loose

    return entries


def read_index(path=None):
    """Return the fields above the first heading: map_count and latest."""
    path = registry_path(path)

    try:
        with open(path, encoding='ascii') as handle:
            lines = handle.readlines()
    except OSError:
        return {}

    index = {}

    for line in lines:
        if _SECTION.match(line):
            break

        field = _FIELD.match(line)

        if field:
            index[field.group('key')] = field.group('value')

    return index


def latest_source(path=None):
    """
    Which source was written most recently.

    The `latest:` line is the authority, because two saves inside the same
    second are a tie that saved_unix cannot break. The timestamps are only a
    fallback, for a file written before that line existed.
    """
    entries = read_all(path)

    if not entries:
        return None

    recorded = read_index(path).get('latest')

    if recorded in entries:
        return recorded

    def stamp(item):
        try:
            return int(item[1].get('saved_unix', 0))
        except ValueError:
            return 0

    return max(entries.items(), key=stamp)[0]


def read_registry(path=None, source=None):
    """
    Return one map's fields, or an empty dict if it is not recorded.

    With no source, the most recently saved map is returned. Callers that
    care which robot they are - the cleaning launch does - should pass one.
    """
    entries = read_all(path)

    if not entries:
        return {}

    if source is None:
        source = latest_source(path)

    return entries.get(source, {})


def saved_map_file(path=None, source=None):
    """Return a saved map's .yaml path from map.md, or None if unusable."""
    map_file = read_registry(path, source).get('map_file')

    if not map_file or not os.path.isfile(map_file):
        return None

    return map_file


def write_registry(path, map_name, yaml_path, pgm_path, source=DEFAULT_SOURCE):
    """
    Record the map that was just saved, leaving the other sources alone.

    The whole file is rewritten each time, but from the entries already in it,
    so a sim save keeps the hardware entry and the other way round.
    """
    now = time.time()

    entries = read_all(path)
    entries[source] = {
        'map_name': map_name,
        'source': source,
        'map_file': yaml_path,
        'map_image': pgm_path,
        'saved_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
        'saved_unix': str(int(now)),
    }

    # Newest first, so the map you just made is the one at the top. The
    # source just written wins any tie, since a save inside the same second
    # as another one would otherwise order arbitrarily.
    order = sorted(
        entries,
        key=lambda name: (name != source,
                          -int(entries[name].get('saved_unix') or 0)),
    )

    text = _HEADER
    text += '\n'
    text += f'- **map_count:** {len(entries)}\n'
    text += f'- **latest:** {source}\n'

    for name in order:
        text += f'\n## {name}\n\n'
        for key in ('map_name', 'source', 'map_file', 'map_image',
                    'saved_at', 'saved_unix'):
            value = entries[name].get(key)
            if value is not None:
                text += f'- **{key}:** {value}\n'

    # The registry sits beside the maps; make sure that folder exists before
    # the first save. Only here - reading never creates anything.
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)

    tmp = path + '.tmp'

    with open(tmp, 'w', encoding='ascii') as handle:
        handle.write(text)

    os.replace(tmp, path)


def count_maps(path=None):
    """How many distinct maps map.md records."""
    return len(read_all(path))
