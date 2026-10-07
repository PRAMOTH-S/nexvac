#!/usr/bin/env bash
# Send this workspace's SOURCE to the Raspberry Pi, then build it there.
#
#   ./push.sh                    sync CHANGED files, rebuild only what moved
#   ./push.sh --full             sync, then rebuild everything (the old behaviour)
#   ./push.sh --dry              show what would be sent, send nothing
#   ./push.sh --no-build         sync only, build nothing
#   ./push.sh --delete           also remove files on the Pi that are gone here
#
# Rebuilding only what moved is the default because a full build of all 13
# packages costs ~40 s on the Pi, and a one-file edit to one package does not
# need the other twelve recompiled. rsync already tells us exactly which files
# it sent, so the package list is derived from that rather than guessed. Use
# --full after changing anything shared (a message definition, a dependency,
# or anything in an installed interface package), where a stale sibling build
# would silently keep the old headers.
#   ./push.sh --mine             overwrite even files newer on the Pi
#   ./push.sh --pull-update      only fetch the Pi's Docs/UPDATE.md and merge it
#                                in here; push nothing. Do this BEFORE editing it.
#   ./push.sh --last             only show the last pushes; send nothing
#   PI_HOST=10.42.0.53 ./push.sh
#   PUSH_BY=Varun ./push.sh      name recorded in the push history (asked once
#                                and remembered in .push_by otherwise)
#
# WHAT WAS PUSHED, AND WHEN
#
# Every push appends one line to .push_history on the Pi (shared, so both of
# you see each other's pushes) and writes the same line to .last_push here:
# time, who, from which machine, how many files, which packages. Each run
# starts by printing the Pi's last few pushes and the files changed HERE since
# this machine last pushed - so "did I push that already?" has an answer.
# Neither file is ever synced; each side keeps its own.
#
# DOCS/UPDATE.MD IS MERGED, NOT OVERWRITTEN
#
# Both of you add entries to it. Plain rsync keeps whichever FILE is newer, so
# the second person to push either lost the first person's entry or never sent
# their own. Before syncing, this fetches the Pi's copy and merges it entry by
# entry (tools/merge_update_md.py): entries only on the Pi are added here, so
# the copy that goes back up has both. If one entry was edited differently on
# each side the push STOPS - pick the right text by hand, then push again.
#
# SHARED WORKSPACE - READ THIS
#
# Someone else works on this robot too, so the default is deliberately
# cautious:
#
#   * rsync runs with --update, so a file that is NEWER on the Pi is left
#     alone. If your colleague edited something directly on the robot, this
#     push will not silently destroy it. The skipped files are listed. Use
#     --mine when you genuinely mean "my copy wins".
#   * --delete is never implied, and still asks before removing anything.
#   * Only the packages you actually changed are rebuilt, so his build of
#     everything else is left intact.
#   * If the robot is RUNNING, you are warned before rebuilding: colcon
#     replaces files under live nodes, and with --symlink-install a running
#     process can pick up half-written code.
#
# build/ install/ log/ are NEVER sent. The Pi is aarch64 and this machine is
# probably x86_64, so copying compiled artifacts across produces exactly the
# "Relocations in generic ELF (EM: 183)" and stale-CMakeCache failures that
# build.sh has to clean up. Source goes over; the Pi compiles its own.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

PI_HOST="${PI_HOST:-10.42.0.241}"
PI_USER="${PI_USER:-Scrapify}"
PI_DIR="${PI_DIR:-~/Pramoth/nexva_ws}"

TARGET="$PI_USER@$PI_HOST"

DRY=false
BUILD=true
DELETE=false
FULL=false
MINE=false
PULL_UPDATE_ONLY=false
LAST_ONLY=false
for arg in "$@"; do
    case "$arg" in
        --dry)      DRY=true ;;
        --no-build) BUILD=false ;;
        --delete)   DELETE=true ;;
        --full)     FULL=true ;;
        --mine)     MINE=true ;;
        --pull-update) PULL_UPDATE_ONLY=true ;;
        --last)     LAST_ONLY=true ;;
        -h|--help)  sed -n '2,62p' "$0"; exit 0 ;;
        *) echo "[push] unknown option: $arg"; exit 1 ;;
    esac
done

# One SSH connection for the whole run. The Pi takes a password, and this
# script talks to it up to seven times (history, UPDATE.md, rsync, chmod,
# process check, build, record) - without multiplexing that is seven prompts.
SSH_CTL="$HOME/.ssh/cm-nexva-%r@%h:%p"
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$SSH_CTL" -o ControlPersist=120)
SSH=(ssh "${SSH_OPTS[@]}")
RSH="ssh ${SSH_OPTS[*]}"

HISTORY_FILE=".push_history"     # on the Pi, in $PI_DIR, appended
LAST_FILE="$WS/.last_push"       # here, overwritten
BY_FILE="$WS/.push_by"           # here: the name to record

# Compiled output, caches, the second workspace, and node_modules. Everything
# here is either machine-specific, regenerable, or enormous.
EXCLUDES=(
    --exclude 'build/'
    --exclude 'install/'
    --exclude 'log/'
    --exclude '__pycache__/'
    --exclude '*.pyc'
    --exclude '.git/'
    --exclude 'simluationsequnce/'
    --exclude 'robot_zigzag_ui/'
    --exclude '*.zip'
    --exclude '*.png'
    --exclude '.vscode/'
    --exclude '/Docs/UPDATE.pi.md'
    --exclude '/.last_push'
    --exclude '/.push_by'
    --exclude "/$HISTORY_FILE"
)

echo "[push] from  $WS"
echo "[push] to    $TARGET:$PI_DIR"
echo "[push] not sending: build/ install/ log/ .git/ simluationsequnce/ robot_zigzag_ui/"
echo

if ! timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PI_HOST/22" 2>/dev/null; then
    echo "[push] ERROR: nothing listening on $PI_HOST:22"
    echo "[push]   is it on?      ping -c1 $PI_HOST"
    echo "[push]   right address? PI_HOST=... ./push.sh"
    exit 1
fi

# ---- What has been pushed already --------------------------------------------
echo "[push] last pushes to the Pi (newest last, from $PI_DIR/$HISTORY_FILE):"
LAST_PI="$("${SSH[@]}" "$TARGET" "tail -n 5 $PI_DIR/$HISTORY_FILE 2>/dev/null" | tr -d '\r' || true)"
if [ -n "$LAST_PI" ]; then
    echo "$LAST_PI" | sed 's/^/[push]   /'
else
    echo "[push]   (no record yet - pushes before $HISTORY_FILE existed were not logged)"
fi
if [ -f "$LAST_FILE" ]; then
    echo "[push] this machine last pushed: $(cat "$LAST_FILE")"
    # Files edited here since then: the answer to "did I push that already?".
    # Same exclusions as the sync, so this lists exactly what could go next.
    CHANGED_SINCE="$(find . \( -path ./build -o -path ./install -o -path ./log \
                         -o -path ./.git -o -path ./simluationsequnce \
                         -o -path ./robot_zigzag_ui -o -name __pycache__ \
                         -o -path ./.vscode \) -prune -o -type f -newer "$LAST_FILE" \
                         ! -name '*.pyc' ! -name '*.zip' ! -name '*.png' \
                         ! -path ./.last_push ! -path ./.push_by -print \
                     | sed 's|^\./||' | sort)"
    if [ -n "$CHANGED_SINCE" ]; then
        echo "[push] changed here since then ($(echo "$CHANGED_SINCE" | wc -l) files):"
        echo "$CHANGED_SINCE" | head -n 30 | sed 's/^/[push]   /'
        [ "$(echo "$CHANGED_SINCE" | wc -l)" -gt 30 ] && echo "[push]   ..."
    else
        echo "[push] nothing changed here since then"
    fi
else
    echo "[push] this machine has no push on record yet"
fi
echo
[ "$LAST_ONLY" = true ] && exit 0

# ---- Docs/UPDATE.md: take the Pi's entries before sending ours ---------------
# Runs on --dry too (report only), so a dry run shows what the merge would do.
UPDATE_MD="$WS/Docs/UPDATE.md"
PI_UPDATE="$(mktemp)"
# Exit 7 means "no such file"; any other failure is the connection, and must
# not be mistaken for "nothing to merge".
set +e
"${SSH[@]}" "$TARGET" "f=$PI_DIR/Docs/UPDATE.md; [ -f \$f ] || exit 7; cat \$f" > "$PI_UPDATE"
FETCH_RC=$?
set -e
if [ "$FETCH_RC" -ne 0 ] && [ "$FETCH_RC" -ne 7 ]; then
    echo "[push] STOPPED: could not read Docs/UPDATE.md from the Pi (ssh exit $FETCH_RC)"
    rm -f "$PI_UPDATE"; exit 1
fi
if [ "$FETCH_RC" -eq 0 ] && [ -s "$PI_UPDATE" ]; then
    echo "[push] merging the Pi's Docs/UPDATE.md into this one..."
    MERGE_ARGS=("$UPDATE_MD" "$PI_UPDATE")
    [ "$DRY" != true ] && MERGE_ARGS+=(--write)
    set +e
    python3 "$WS/tools/merge_update_md.py" "${MERGE_ARGS[@]}"
    MERGE_RC=$?
    set -e
    case "$MERGE_RC" in
        0) echo "[push]   nothing new on the Pi" ;;
        1) [ "$DRY" = true ] && echo "[push]   (dry run: not written)" ;;
        2) cp "$PI_UPDATE" "$WS/Docs/UPDATE.pi.md"
           echo "[push] STOPPED. The Pi's copy is saved as Docs/UPDATE.pi.md:"
           echo "[push]   diff Docs/UPDATE.md Docs/UPDATE.pi.md"
           echo "[push]   fix Docs/UPDATE.md, delete UPDATE.pi.md, push again"
           rm -f "$PI_UPDATE"; exit 1 ;;
        *) echo "[push] STOPPED: could not merge UPDATE.md (see above)"
           rm -f "$PI_UPDATE"; exit 1 ;;
    esac
else
    echo "[push] the Pi has no Docs/UPDATE.md yet - nothing to merge"
fi
rm -f "$PI_UPDATE"
echo
if [ "$PULL_UPDATE_ONLY" = true ]; then
    echo "[push] --pull-update: Docs/UPDATE.md is current with the Pi. Nothing pushed."
    exit 0
fi

RSYNC=(rsync -az -e "$RSH" --info=stats1,progress2 --human-readable "${EXCLUDES[@]}")
if [ "$MINE" != true ]; then
    # Do not clobber work done directly on the Pi by someone else.
    RSYNC+=(--update)
fi

if [ "$DELETE" = true ]; then
    # --delete removes files on the Pi. Always show what that means first.
    echo "[push] --delete: previewing removals on the Pi"
    rsync -an -e "$RSH" --delete "${EXCLUDES[@]}" "$WS/" "$TARGET:$PI_DIR/" \
        | grep '^deleting ' || echo "[push]     (nothing would be deleted)"
    echo
    if [ "$DRY" != true ]; then
        read -r -p "[push] proceed with those deletions? [y/N]: " reply
        case "$reply" in
            y|Y|yes) RSYNC+=(--delete) ;;
            *) echo "[push] continuing WITHOUT --delete" ;;
        esac
    fi
fi

if [ "$DRY" = true ]; then
    echo "[push] DRY RUN - nothing will be written"
    echo
    rsync -anv -e "$RSH" --stats "${EXCLUDES[@]}" "$WS/" "$TARGET:$PI_DIR/"
    exit 0
fi

"${SSH[@]}" "$TARGET" "mkdir -p $PI_DIR"
# --out-format makes rsync name every file it transfers; that list is what
# decides which packages need rebuilding. Printed to the terminal as well, so
# the operator still sees progress.
# rsync's output goes to a file via plain tee and is filtered AFTER rsync
# exits: a `tee >(...)` filter runs in the background and could still be
# writing the list when the next line reads it.
SENT_LIST="$(mktemp)"
RSYNC_OUT="$(mktemp)"
trap 'rm -f "$SENT_LIST" "$RSYNC_OUT"' EXIT
set +e
"${RSYNC[@]}" --out-format='SENT %n' "$WS/" "$TARGET:$PI_DIR/" | tee "$RSYNC_OUT"
RSYNC_RC=${PIPESTATUS[0]}
set -e
# Without this a failed sync carried on - and would now be RECORDED as a push.
if [ "$RSYNC_RC" -ne 0 ]; then
    echo "[push] STOPPED: rsync failed (exit $RSYNC_RC). Nothing recorded."
    exit 1
fi
grep '^SENT ' "$RSYNC_OUT" | sed 's/^SENT //' > "$SENT_LIST" || true

# rsync preserves the executable bit, but only if it was set here - and on
# this laptop it was not. The mode manager EXECUTES launch/realbot/robot_*.sh
# directly, so a missing bit there is "Permission denied" on every mode switch
# from the web UI. Every script, not a hand-kept list that goes stale.
"${SSH[@]}" "$TARGET" "cd $PI_DIR && chmod +x *.sh launch/realbot/*.sh tools/*.sh 2>/dev/null || true"

echo
echo "[push] source is on the Pi."

# ---- Record this push ---------------------------------------------------------
# Written as soon as the sync succeeded: that is what "pushed" means. Whether
# it was then built is a separate question the build output answers.
PUSH_BY="${PUSH_BY:-}"
if [ -z "$PUSH_BY" ] && [ -s "$BY_FILE" ]; then
    PUSH_BY="$(head -n1 "$BY_FILE")"
fi
if [ -z "$PUSH_BY" ] && [ -t 0 ]; then
    read -r -p "[push] your name, for the push history (asked once): " PUSH_BY
    [ -n "$PUSH_BY" ] && echo "$PUSH_BY" > "$BY_FILE"
fi
PUSH_BY="${PUSH_BY:-unknown}"
SENT_N="$(grep -cv '/$' "$SENT_LIST" || true)"
SENT_PKGS="$(sed -n 's|^src/\([^/]*\)/.*|\1|p' "$SENT_LIST" 2>/dev/null | sort -u | tr '\n' ' ' | sed 's/ *$//')"
SENT_OTHER="$(grep -v '^src/' "$SENT_LIST" 2>/dev/null | grep -v '/$' | head -n 5 | tr '\n' ' ' | sed 's/ *$//')"
RECORD="$(date '+%Y-%m-%d %H:%M:%S %z') | by $PUSH_BY | from $(whoami)@$(hostname)"
RECORD+=" | $SENT_N files | packages: ${SENT_PKGS:-none}"
[ -n "$SENT_OTHER" ] && RECORD+=" | other: $SENT_OTHER"
echo "$RECORD" > "$LAST_FILE"
echo "$RECORD" | "${SSH[@]}" "$TARGET" "cat >> $PI_DIR/$HISTORY_FILE" \
    || echo "[push] WARNING: could not append to $HISTORY_FILE on the Pi"
echo "[push] recorded: $RECORD"

# Is the robot live right now? Rebuilding swaps files under running nodes, and
# with --symlink-install the installed file IS the source file, so a process
# can read half-written code. Worth one question rather than a mystery crash
# on someone else's run.
RUNNING="$("${SSH[@]}" "$TARGET" "pgrep -c -f 'install/nexva_|micro_ros_agent' 2>/dev/null || echo 0" 2>/dev/null | tr -d '\r')"
if [ "$BUILD" = true ] && [ "${RUNNING:-0}" -gt 0 ] 2>/dev/null; then
    echo
    echo "[push] WARNING: $RUNNING robot process(es) are running on the Pi right now."
    echo "[push]   Rebuilding replaces files underneath them. If someone else is"
    echo "[push]   driving, this can crash their run."
    if [ -t 0 ]; then
        read -r -p "[push] rebuild anyway? [y/N]: " reply
        case "$reply" in
            y|Y|yes) ;;
            *) echo "[push] skipping the build - source is synced, build it later"; BUILD=false ;;
        esac
    else
        echo "[push]   (not a terminal - skipping the build to be safe; use --no-build"
        echo "[push]    to silence this, or rebuild on the Pi yourself)"
        BUILD=false
    fi
fi

if [ "$BUILD" = true ]; then
    CHANGED_PKGS=""
    if [ "$FULL" != true ] && [ -s "$SENT_LIST" ]; then
        # src/<pkg>/... -> <pkg>. Anything sent from OUTSIDE src/ (a root
        # script, tools/, Docs/) needs no colcon build at all, so it simply
        # contributes no package and is skipped.
        CHANGED_PKGS="$(sed -n 's|^src/\([^/]*\)/.*|\1|p' "$SENT_LIST" \
                        | sort -u | tr '\n' ' ')"
    fi

    if [ "$FULL" = true ]; then
        echo "[push] --full: rebuilding everything"
        "${SSH[@]}" -t "$TARGET" "cd $PI_DIR && ./build.sh"
    elif [ -n "$CHANGED_PKGS" ]; then
        echo "[push] rebuilding only what changed: $CHANGED_PKGS"
        echo "[push]   (use --full if you touched a message, a dependency, or"
        echo "[push]    anything other packages compile against)"
        echo
        "${SSH[@]}" -t "$TARGET" "cd $PI_DIR && ./build.sh $CHANGED_PKGS"
    else
        echo "[push] nothing under src/ changed - no rebuild needed"
        if [ -s "$SENT_LIST" ]; then
            echo "[push]   (files were still synced: $(wc -l < "$SENT_LIST") of them)"
        fi
    fi
fi

echo
echo "[push] done. On the Pi:"
echo "[push]     ssh $TARGET"
echo "[push]     cd $PI_DIR && ./robotbring.sh  # terminal 1: bringup"
echo "[push]     cd $PI_DIR && ./robotnav.sh    # terminal 2: Nav2"
echo "[push]     cd $PI_DIR && ./web.sh         # terminal 3: web UI"
