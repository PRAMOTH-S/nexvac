"""Delete-safety for saved maps. Uses a temp maps root, never ~/nexva_maps."""

import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import nexva_explore                                     # noqa: F401
except ImportError:      # unsourced shell: use the sibling source package
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), 'nexva_explore'))

from nexva_explore import map_library, map_registry          # noqa: E402,F401
from nexva_web import map_admin                              # noqa: E402

PARTS = ('.yaml', '.pgm', '.posegraph', '.data',
         '.as_mapped.pgm', '.as_mapped.yaml')


@pytest.fixture
def root(monkeypatch):
    with tempfile.TemporaryDirectory() as tmp:
        d = os.path.join(tmp, 'hardware')
        os.makedirs(d)
        for name in ('kitchen', 'hall'):
            for p in PARTS:
                open(os.path.join(d, name + p), 'w').write('x')
        open(os.path.join(tmp, 'outside.yaml'), 'w').write('x')
        os.makedirs(os.path.join(d, 'subdir'))
        # a bundled map, as nexva_navigation_maps() would report it
        nav = os.path.join(tmp, 'navsrc')
        os.makedirs(nav)
        for p in ('.yaml', '.pgm'):
            open(os.path.join(nav, 'bundled' + p), 'w').write('x')
        monkeypatch.setattr(map_registry, 'nexva_navigation_maps', lambda: [
            {'name': 'bundled', 'yaml': os.path.join(nav, 'bundled.yaml'),
             'pgm': os.path.join(nav, 'bundled.pgm')}])
        map_registry.write_registry(os.path.join(tmp, 'map.md'), 'kitchen',
                                    os.path.join(d, 'kitchen.yaml'),
                                    os.path.join(d, 'kitchen.pgm'),
                                    'hardware')
        yield tmp


def files(root):
    return sorted(os.listdir(os.path.join(root, 'hardware')))


@pytest.mark.parametrize('bad', [
    'nope', '../outside', '..', '../../etc/passwd', 'a/b', '/etc/passwd',
    '', None, '.hidden', 'kitchen.as_mapped', 'kitchen/../hall',
    'kitchen\x00', 'subdir'])
def test_unlisted_or_unsafe_names_refused(root, bad):
    before = files(root)
    with pytest.raises(map_admin.MapDeleteError):
        map_admin.delete_map(bad, 'hardware', root)
    assert files(root) == before
    assert os.path.exists(os.path.join(root, 'outside.yaml'))


def test_bundled_map_refused(root):
    with pytest.raises(map_admin.MapDeleteError, match='bundled'):
        map_admin.delete_map('bundled', 'hardware', root)
    assert os.path.exists(os.path.join(root, 'navsrc', 'bundled.yaml'))
    assert os.path.exists(os.path.join(root, 'navsrc', 'bundled.pgm'))


def test_active_mission_map_refused_but_other_map_ok(root):
    mode = {'mode': 'clean', 'map': 'kitchen'}
    with pytest.raises(map_admin.MapDeleteError, match='in use'):
        map_admin.delete_map('kitchen', 'hardware', root, mode)
    assert 'kitchen.yaml' in files(root)
    out = map_admin.delete_map('hall', 'hardware', root, mode)
    assert out['ok'] and out['mission_state_known']


def test_unknown_mission_state_is_reported(root):
    out = map_admin.delete_map('hall', 'hardware', root, {})
    assert out['ok'] and out['mission_state_known'] is False


def test_only_that_maps_files_removed_and_registry_fixed(root):
    out = map_admin.delete_map('kitchen', 'hardware', root,
                               {'mode': 'idle', 'map': 'kitchen'})
    assert sorted(out['deleted']) == sorted('kitchen' + p for p in PARTS)
    left = files(root)
    assert left == sorted(['hall' + p for p in PARTS] + ['subdir'])
    assert os.path.exists(os.path.join(root, 'outside.yaml'))
    assert os.path.isdir(os.path.join(root, 'hardware', 'subdir'))
    reg = map_registry.read_all(os.path.join(root, 'map.md'))
    assert reg['hardware']['map_name'] == 'hall'      # re-pointed, not dangling


def test_registry_removed_when_last_map_goes(root):
    map_admin.delete_map('hall', 'hardware', root)
    map_admin.delete_map('kitchen', 'hardware', root)
    assert map_registry.read_all(os.path.join(root, 'map.md')) == {}
