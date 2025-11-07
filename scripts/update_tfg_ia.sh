#!/usr/bin/env bash
set -euo pipefail

REMOTE_NAME="tfg-ia-remote"
BRANCH_NAME="main"
PREFIX_DIR="tfg-ia"

# Ensure we're at repo root
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_ROOT" ]]; then
  echo "Not inside a Git repository. Abort." >&2
  exit 1
fi
cd "$REPO_ROOT"

# Fetch and pull subtree
echo "Fetching from $REMOTE_NAME..."
git fetch "$REMOTE_NAME" "$BRANCH_NAME"

echo "Pulling subtree into $PREFIX_DIR from $REMOTE_NAME/$BRANCH_NAME..."
git subtree pull --prefix="$PREFIX_DIR" "$REMOTE_NAME" "$BRANCH_NAME" -m "chore(subtree): sync $PREFIX_DIR from $REMOTE_NAME/$BRANCH_NAME"

echo "Done."
#!/usr/bin/env bash
set -euo pipefail

REMOTE_NAME="tfg-ia-remote"
BRANCH_NAME="main"
PREFIX_DIR="tfg-ia"

# Ensure we're at repo root
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_ROOT" ]]; then
  echo "Not inside a Git repository. Abort." >&2
  exit 1
fi
cd "$REPO_ROOT"

# Fetch and pull subtree
echo "Fetching from $REMOTE_NAME..."
git fetch "$REMOTE_NAME" "$BRANCH_NAME"

echo "Pulling subtree into $PREFIX_DIR from $REMOTE_NAME/$BRANCH_NAME..."
git subtree pull --prefix="$PREFIX_DIR" "$REMOTE_NAME" "$BRANCH_NAME" -m "chore(subtree): sync $PREFIX_DIR from $REMOTE_NAME/$BRANCH_NAME"

echo "Done."
