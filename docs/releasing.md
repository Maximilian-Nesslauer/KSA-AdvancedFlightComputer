# Releasing

Every release is built by CI; nobody builds or uploads a zip by hand.
A nightly is built from `main`, and a stable release is a nightly promoted without being built again, so it holds exactly the bytes that were tested.

## Nightlies

`.github/workflows/nightly.yml` runs every night at 03:23 UTC, and from the Actions tab with "Run workflow".
It builds only when `main` has moved since the newest published nightly, whether it runs on schedule or by hand.
A run that fails publishes nothing, so the next run builds that commit again.
It runs the same jobs as `ci.yml`, then publishes the mod zip as a GitHub pre-release.

The tag is the patch after the highest stable release, plus the date, for example `v0.8.2-nightly.20260925`.
A second nightly on the same day gets a counter, `v0.8.2-nightly.20260925.2`.
SemVer sorts every nightly below the release it leads to.

A nightly is never marked Latest, so the repository page and `/releases/latest` keep pointing at the stable release.
The content index stamps a `-nightly` version as a dev release, so Borea offers nightlies only on its `dev` channel.
`spacedock-publish.yml` skips pre-releases, so SpaceDock never gets a nightly.

Only the newest five nightlies are kept, and older ones are deleted with their tags.
The index still lists a deleted nightly, so installing that exact version in Borea fails, while Borea's newest dev release is always one that exists.

Each nightly carries the zip, named after the version, and `SHA256SUMS.txt`.
The zip has a build provenance attestation from the nightly workflow.

## Stable releases

1. Optionally, write a summary in `.github/release-notes/<version>.md` on `main`, for example `.github/release-notes/0.8.2.md`.
   It goes at the top of the release notes, above the generated change list.
2. In the Actions tab, run "Promote nightly".
   Leave both fields empty to promote the newest nightly to its own version without `-nightly`, or name the nightly and the version, for example to release `0.9.0`.
3. The workflow checks that the nightly's commit is on `main` and that the version is new and above the latest stable release.
   It checks the zip against the nightly's `SHA256SUMS.txt` and its attestation, then creates a draft release with the same zip.
4. Open the draft, check the notes, and add or edit the summary there if there is none.
5. Publish the draft.
   Publishing creates the tag on the nightly's commit and makes the release Latest.
   `spacedock-publish.yml` then uploads it to SpaceDock, and the content index stamps it as stable.

Running "Promote nightly" again for the same version replaces its draft.
A published release is final: a fix needs a new version.

The draft is published by hand on purpose.
A release that a workflow publishes with its own token does not start other workflows, so the SpaceDock upload would not run.

## Release notes

`build/ci/write-release-notes.sh` writes the notes of both kinds, in this order:

- the summary: a fixed note for a nightly, the hand-written file for a stable release;
- GitHub's generated change list, since the previous nightly or the previous stable release;
- the KSA build from `TestedGameVersion` in `Mod.cs` at the released commit;
- installation;
- the files with their SHA-256;
- how to check the attestation with `gh attestation verify`.

The SpaceDock changelog is the release body, so it gets the same text.
