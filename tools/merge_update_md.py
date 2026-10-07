#!/usr/bin/env python3
"""Merge the Pi's Docs/UPDATE.md into this machine's copy, entry by entry.

    python3 tools/merge_update_md.py LOCAL REMOTE            report only
    python3 tools/merge_update_md.py LOCAL REMOTE --write    also rewrite LOCAL

Two people add entries to UPDATE.md, one on each machine, and push.sh syncs
with `rsync --update` - newest FILE wins. So whoever pushed second silently
dropped the other person's entry, or never sent their own. Merging by ENTRY
instead of by file keeps both.

An entry is a `## ` heading after the `---` that ends the preamble, up to the
next such heading. Headings inside ``` fences are content, not entries (the
"How to add an entry" template has one). Entries are matched on their heading
line.

  * on the Pi only     -> inserted into LOCAL, in date order (newest first)
  * here only          -> kept; these are what the next push will send
  * both, same text    -> nothing to do
  * both, text differs -> CONFLICT. Nothing is written; resolve by hand. Picking
                          a side automatically is exactly what lost entries.

Exit status: 0 nothing to take from the Pi, 1 Pi entries merged (or would be,
without --write), 2 conflict, 3 could not parse.
"""

import re
import sys

DATE = re.compile(r'^## (\d{4}-\d{2}-\d{2})')


def parse(text):
    """Return (preamble_lines, [(heading, body_lines)])."""
    lines = text.splitlines()
    in_fence = False
    preamble_end = None
    for i, line in enumerate(lines):
        if line.startswith('```'):
            in_fence = not in_fence
        elif not in_fence and line.strip() == '---':
            preamble_end = i + 1
            break
    if preamble_end is None:
        raise ValueError('no "---" line ending the preamble')

    preamble = lines[:preamble_end]
    entries = []
    in_fence = False
    for line in lines[preamble_end:]:
        if line.startswith('```'):
            in_fence = not in_fence
        if not in_fence and line.startswith('## '):
            entries.append((line.rstrip(), [line]))
        elif entries:
            entries[-1][1].append(line)
        elif line.strip():
            raise ValueError('text between "---" and the first entry: %r' % line)
    return preamble, entries


def norm(body):
    """Compare entries ignoring trailing whitespace and blank-line padding."""
    return '\n'.join(l.rstrip() for l in body).strip()


def date_of(heading):
    m = DATE.match(heading)
    return m.group(1) if m else ''


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 3
    local_path, remote_path = argv[1], argv[2]
    write = '--write' in argv[3:]

    try:
        with open(local_path, encoding='utf-8') as f:
            local_pre, local = parse(f.read())
        with open(remote_path, encoding='utf-8') as f:
            remote_pre, remote = parse(f.read())
    except (OSError, ValueError) as e:
        print('[update.md] cannot parse: %s' % e)
        return 3

    local_by = {h: b for h, b in local}
    remote_by = {h: b for h, b in remote}

    conflicts = [h for h in local_by
                 if h in remote_by and norm(local_by[h]) != norm(remote_by[h])]
    pi_only = [(h, b) for h, b in remote if h not in local_by]
    here_only = [h for h, _ in local if h not in remote_by]

    if norm(local_pre) != norm(remote_pre):
        print('[update.md] note: the header/instructions above "---" differ; '
              'keeping this machine\'s')
    for h in here_only:
        print('[update.md] only here (will be pushed): %s' % h)
    for h, _ in pi_only:
        print('[update.md] only on the Pi (merging in): %s' % h)

    if conflicts:
        for h in conflicts:
            print('[update.md] CONFLICT - same entry, different text: %s' % h)
        print('[update.md] nothing written. Compare the two copies, make this '
              'machine\'s entry the right one, then push again.')
        return 2

    if not pi_only:
        return 0

    # Insert each Pi-only entry above the first local entry that is older, so
    # the file stays newest-first. Same date: the Pi's entry goes on top of
    # that day's group, since there is no clock finer than a day to go by.
    merged = list(local)
    for h, b in pi_only:
        d = date_of(h)
        pos = len(merged)
        for i, (mh, _) in enumerate(merged):
            if date_of(mh) <= d:
                pos = i
                break
        merged.insert(pos, (h, b))

    if write:
        out = list(local_pre)
        for _, body in merged:
            # Exactly one blank line between entries, whatever each side had.
            while body and not body[-1].strip():
                body = body[:-1]
            out.append('')
            out.extend(body)
        with open(local_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(out) + '\n')
        print('[update.md] merged %d entr%s from the Pi into %s'
              % (len(pi_only), 'y' if len(pi_only) == 1 else 'ies', local_path))
    return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
