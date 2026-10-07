#!/usr/bin/env bash
# Commit everything in this workspace and push it to GitHub in one go.
#
#   ./gitpush.sh                      list changes, ask what you changed, commit, push
#   ./gitpush.sh "fixed PID gains"    commit with this message (no question), push
#   ./gitpush.sh --dry                show what would be committed, push nothing
#
# This is NOT push.sh. push.sh sends source to the Pi; this sends it to
# https://github.com/PRAMOTH-S/nexva_ws (private). Override with
#   GIT_REMOTE_URL=https://github.com/PRAMOTH-S/other.git ./gitpush.sh
#
# Auth comes from the GitHub CLI (`gh auth login`), so no token lives in this
# repo. build/, install/ and log/ are kept out by .gitignore.
#
# It never force-pushes. If GitHub has commits this machine does not, it
# rebases on top of them first; on a conflict it stops and tells you.

set -euo pipefail

REMOTE_URL="${GIT_REMOTE_URL:-https://github.com/PRAMOTH-S/nexvac.git}"
BRANCH="${GIT_BRANCH:-main}"
cd "$(dirname "$(readlink -f "$0")")"

DRY=0
MSG=""
for arg in "$@"; do
    case "$arg" in
        --dry) DRY=1 ;;
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) MSG="$arg" ;;
    esac
done

if ! gh auth status >/dev/null 2>&1; then
    echo "Not logged in to GitHub. Run:  gh auth login" >&2
    exit 1
fi

# First run: this folder was never a git repo, but GitHub already has its
# history. Attach to that history instead of starting a new one, so the push
# is an ordinary commit on top and not a rewrite.
if [ ! -d .git ]; then
    echo "First run: connecting this folder to $REMOTE_URL"
    git init -q -b "$BRANCH"
    git remote add origin "$REMOTE_URL"
    git config credential.https://github.com.helper '!gh auth git-credential'
    if git fetch -q origin "$BRANCH" 2>/dev/null; then
        git reset -q "origin/$BRANCH"     # adopt history, keep files on disk as they are
        git branch -q --set-upstream-to="origin/$BRANCH"
    fi
fi

git add -A
if git diff --cached --quiet; then
    echo "Nothing changed since the last commit."
else
    echo "Changes to commit:"
    git diff --cached --stat | tail -25
    if [ "$DRY" = 1 ]; then
        git reset -q
        echo "(--dry: nothing committed or pushed)"
        exit 0
    fi
    if [ -z "$MSG" ] && [ -t 0 ]; then
        echo
        read -r -p "What did you change today? (Enter = automatic message) " MSG
    fi
    git commit -q -m "${MSG:-Update $(date '+%Y-%m-%d %H:%M') from $(hostname)}"
fi
[ "$DRY" = 1 ] && exit 0

if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
    if ! git pull -q --rebase origin "$BRANCH"; then
        echo >&2
        echo "GitHub has changes that conflict with yours. Nothing was pushed." >&2
        echo "Fix the files listed above, then: git add -A && git rebase --continue && ./gitpush.sh" >&2
        exit 1
    fi
fi

git push -q -u origin "$BRANCH"
echo "Pushed: $(git log -1 --format='%h %s')"
echo "        ${REMOTE_URL%.git}/commits/$BRANCH"
