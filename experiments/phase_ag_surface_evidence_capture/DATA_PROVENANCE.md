# Existing-data evidence audit

Executed first: read-only inventory of local map collection, recording metadata, candidate reference CSVs and existing provenance/semantics reports. All9 candidate PCDs were fully SHA256-hashed, along with8 recording metadata files,3 earlier derived-reference CSVs,2 later raw/built-reference CSVs and selected supporting documents. Original paths, complete headers, file mtimes, clock bounds and reference fields are LOCAL ONLY. Public non-reconstructive identities below are not acquisition certificates.

|Alias|Bytes|SHA256|Independent acquisition / common datum|
|---|---:|---|---|
|M01|52955360|`6f0b10c52fa9bbdc02fc83646ba52d85cfa69ccab129444098946c028bc78c92`|UNVERIFIED|
|M02|1076917202|`8ff42fd25add3a59bf68e413b2510dbb91bd48bdc421737d2735dcdb4055345c`|UNVERIFIED|
|M03|1076921104|`8561cd2254ed35164134a3146e89dfcd403643193ff11c84db93fde2b77ad52a`|UNVERIFIED|
|M04|52951517|`0dd39161197fa75da60600b710772dac860c8b61a0101742b336e4db66b19d5a`|UNVERIFIED|
|M05|1930527776|`5d165afc16f22aaad785711a37575673b09587f1d934edb5cb6dc85b2f52de8f`|UNVERIFIED|
|M06|667752070|`976af9e2ffc3ba28fa6741e56b93d209988952b64674c7572a22caa8714d4828`|UNVERIFIED|
|M07|22681552|`e8722a7b62e983a5b3167b30870ed280e51feb0ed08dc7a0a55d0c13ebc9db82`|UNVERIFIED|
|M08|3540904141|`0ebbc14aa6481f5332da96f1bbfa352d67d3ea764d25061120b9cab74b328228`|UNVERIFIED|
|M09|642591493|`7d34abe29e3416400deeb9ddd51c1e549f42e19f4fcbd59568345712fce6d054`|UNVERIFIED|

Nine distinct digests do not prove nine independent maps. Several filenames, cached normal/downsample products, identical point counts and archived crop reports indicate possible transformation families. The existing map comparison utility compares two inputs but does not record independent capture sessions, construction scan hashes, registration constraints or datum/extrinsic calibration. Full PCD headers provide no adequate acquisition chain: VIEWPOINT is not an ENU origin/high-altitude datum or calibration certificate. No map alignment was fitted to manufacture a common frame. Dates in names and filesystem mtimes are not measurement dates.

Existing replay archives contain additional cropped Targets and Sources; their reports explicitly describe crops/downsamples/replays, so these are not promoted to new independent maps. Other recording metadata show stored topic/time coverage, but no verified source-scan-to-map-build exclusion or independent reference chain. Known frozen native Source/Target/AA state/time identities are inherited from AF and checked, not rebuilt. Whether the tested Source or its localization errors participated in map construction remains UNKNOWN, not assumed independent or assumed shared.

The earlier derived reference has a corrected-delay/validation history, but its full temporal coverage ends60.9195s BEFORE this first frozen state. The later raw GPS/built trajectory begins121.6191/121.4940s AFTER it. Prior reports disclose partial reference validity, an estimated/built position chain, unqualified physical time/datum/lever contracts and possible dependent historical attitude sources. A file labeled FULL_TRUTH=PASS describes its original analysis scope, not the present first frame. No new delay fitting, datum alignment, reference pose interpolation beyond coverage or accuracy score was done.

Conclusion: DATA_INSUFFICIENT for real plane covariance calibration and independent first-frame accuracy. This is a bounded evidence audit, not a claim that independent maps can never exist. Recovery requires acquisition IDs/dates and raw scan membership, mapping/localization dependency graphs, fixed frame/origin/vertical datum and extrinsic versions, plus a time-qualified independent surface/pose reference at the tested interval. None may be substituted by two different PCDs or within-map point bootstrap.
