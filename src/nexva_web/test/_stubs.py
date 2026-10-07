"""Shared stand-ins for the bridge tests.

aiohttp is not installed on every machine that runs these (the dev laptop
does not have it), and the tests do not need a real HTTP server anyway: what
is under test is the bridge's own logic - batching, dispatch, the commands. So
`install_aiohttp_stub()` puts a minimal `aiohttp` in sys.modules *if the real
one cannot be imported*, and FakeWS stands in for a WebSocketResponse.
"""

import asyncio
import importlib.util
import json
import os
import sys
import types

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)


def install_aiohttp_stub():
    """Idempotent: safe to call from every test module in one pytest run."""
    if 'aiohttp' in sys.modules:             # real, or already stubbed
        return False
    try:
        if importlib.util.find_spec('aiohttp') is not None:
            return False
    except (ValueError, ImportError):
        pass
    mod = types.ModuleType('aiohttp')
    web = types.ModuleType('aiohttp.web')

    class WSMsgType:
        TEXT = 1
        CLOSE = 8

    class _Any:
        def __init__(self, *a, **k):
            pass

    web.WebSocketResponse = _Any
    web.Application = _Any
    web.Response = _Any
    web.FileResponse = _Any
    web.json_response = lambda *a, **k: (a, k)
    web.run_app = lambda *a, **k: None
    mod.WSMsgType = WSMsgType
    mod.web = web
    sys.modules['aiohttp'] = mod
    sys.modules['aiohttp.web'] = web
    return True


class FakeWS:
    """Records what the server sends. `scripted` are incoming client messages."""

    def __init__(self, scripted=(), stall=False):
        self.sent = []                 # decoded JSON frames, in order
        self.sent_at = []              # loop.time() of each
        self.closed = False
        self.stall = stall             # send_str never completes (stuck phone)
        self._scripted = list(scripted)

    async def send_str(self, text):
        if self.stall:
            await asyncio.sleep(3600)
        self.sent.append(json.loads(text))
        try:
            self.sent_at.append(asyncio.get_running_loop().time())
        except RuntimeError:
            pass

    async def close(self):
        self.closed = True

    def types(self):
        return [m.get('type') for m in self.sent]

    def items(self, kind=None):
        """Every event carried in batch frames, optionally of one type."""
        out = []
        for m in self.sent:
            if m.get('type') == 'batch':
                out.extend(m['items'])
        return [i for i in out if kind is None or i.get('type') == kind]

    # async iteration over the scripted incoming messages (ws_handler uses it)
    def __aiter__(self):
        return self._iter()

    async def _iter(self):
        for delay, payload in self._scripted:
            if delay:
                await asyncio.sleep(delay)
            yield types.SimpleNamespace(type=1, data=json.dumps(payload))
