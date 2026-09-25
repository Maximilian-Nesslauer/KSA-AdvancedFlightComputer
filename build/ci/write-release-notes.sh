#!/usr/bin/env bash
# Writes the body of a nightly or stable GitHub release to stdout, in the order readers need it:
# the hand-written summary, the changes, the tested game build, installation, the downloads with
# their checksums, and how to check where a file came from.
#
# Used by .github/workflows/nightly.yml and promote.yml. Reads everything from the environment:
#   KIND           nightly or stable
#   TAG            the release tag, e.g. v0.8.2 or v0.8.2-nightly.20260925
#   SHA            the commit the release is built from
#   PREVIOUS       the tag the change list starts from; empty lets GitHub pick
#   INTRO          optional file with the hand-written summary
#   DIST           folder holding SHA256SUMS.txt
#   KEEP_NIGHTLIES how many nightlies nightly.yml keeps, for the nightly notes
#   GH_REPO        owner/repo
#   GH_TOKEN       a token that can generate release notes (contents: write)
set -euo pipefail

: "${KIND:?}" "${TAG:?}" "${SHA:?}" "${DIST:?}" "${GH_REPO:?}"

# The game build is read from the commit being released, not from the checkout, because a
# promotion runs on main and releases an older commit.
game=$(gh api -H "Accept: application/vnd.github.raw+json" \
  "repos/$GH_REPO/contents/AdvancedFlightComputer/Mod.cs?ref=$SHA" |
  sed -n 's/.*TestedGameVersion *= *"v\{0,1\}\([^"]*\)".*/\1/p' | head -1)
if [ -z "$game" ]; then
  echo "Could not read TestedGameVersion from AdvancedFlightComputer/Mod.cs at $SHA." >&2
  exit 1
fi

# GitHub's generated change list. Asking for it with the tag and the commit works before the tag
# exists, which a draft needs.
previous_args=()
if [ -n "${PREVIOUS:-}" ]; then
  previous_args=(-f previous_tag_name="$PREVIOUS")
fi
changes=$(gh api --method POST "repos/$GH_REPO/releases/generate-notes" \
  -f tag_name="$TAG" -f target_commitish="$SHA" "${previous_args[@]}" --jq .body) || changes=

if [ "$KIND" = nightly ]; then
  cat <<EOF
Nightly build of \`main\` at $SHA, for testing. It has not been flown as a release, so it can break things that the latest stable release does not.

Only the newest ${KEEP_NIGHTLIES:-5} nightlies are kept. Borea offers nightlies only on its \`dev\` channel, and SpaceDock never gets them.

EOF
elif [ -n "${INTRO:-}" ] && [ -s "$INTRO" ]; then
  cat "$INTRO"
  echo
fi

if [ -n "$changes" ]; then
  # GitHub's list starts with its own "What's Changed" heading.
  printf '%s\n\n' "$changes"
fi

cat <<EOF
## Compatibility
Validated against KSA build version **$game**.

## Installation
Extract into \`Documents\\My Games\\Kitten Space Agency\\mods\\\`, so that the files end up in \`mods\\AdvancedFlightComputer\\\`.
Requires [StarMap](https://github.com/StarMapLoader/StarMap).

You can also download and install it with the mod manager **Borea**: https://ksamodding.github.io/Borea/

## Downloads

| File | SHA-256 |
| --- | --- |
EOF
while read -r sum file; do
  printf '| %s | `%s` |\n' "${file#\*}" "$sum"
done < "$DIST/SHA256SUMS.txt"

cat <<EOF

## Provenance
The zip was built by the nightly workflow of this repository. A promoted stable release carries the same bytes as the nightly it was promoted from, so the same attestation covers it. To check a download:

\`\`\`
gh attestation verify <file> --repo $GH_REPO \\
  --signer-workflow $GH_REPO/.github/workflows/nightly.yml
\`\`\`
EOF
