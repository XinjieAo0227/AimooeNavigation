# v0.1.0 source release preparation

Target: `github.com/XinjieAo0227/AimooeNavigation`, new public repository.
Title: Aimooe Navigation v0.1.0 — LaunchOptimized082 research workflow.
Upstream application provenance: 0.8.2; first public packaging version: 0.1.0.

Source scope: only current LaunchOptimized082 implementation, SDK-free tests/synthetic
examples, bilingual docs, MIT license, CFF, read-only CI and source packaging tools.
Excluded: vendor SDK/drivers, tool definitions, CAD and derived profiles, medical images,
captures/videos, raw logs, original archives, older launchers and version snapshots.

Author/copyright holder: XinjieAo0227, sole author, no institution or co-contributors,
as explicitly confirmed by the author. MIT was explicitly selected by the author.
CFF 1.2.0 schema passes. No DOI, ORCID or release date is invented.
Vendor assets remain excluded because their redistribution rights are unknown.
The adapter is separately authored according to the author’s declaration; static
comparison with supplied vendor Python demos found no identical function bodies.

Validation: 50 SDK-free tests passed; simulated Slicer precision and UI tests passed.
The source archive was tested from an independent extraction. Runtime/scientific code
has not changed in the metadata/license preparation. See VALIDATION.md for details.
Not verified: fresh standalone CPython environment, Linux, real hardware accuracy,
CAD conversion, recording. Remote CI must be checked after the authorized push.

Local asset: `dist/aimooe-navigation-0.1.0.zip`, MANIFEST.json, SHA256SUMS.
Publish only reviewed source files using a new initial history; no original Git metadata exists.
The user authorized public publication under their account in this chat.
No deployment or automatic publishing job exists; pushing source triggers tests only.
Git tag/Release creation will use the verified commit after CI succeeds and after
confirming that those operations are included in the user’s publication scope.
Do not overwrite any existing repository, branch, tag or release.
