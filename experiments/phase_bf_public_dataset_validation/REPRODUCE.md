# BF0 reproduction and stop conditions

Scope: reproduce metadata review only, notdatasetexperiment. Researchcodebaseline da3ea9df0acf34eb5c0808c2e258c9bbb86bafb1. No configurationoradapterimplemented; NO_BF_REPLAY.

Read-onlyenvironmentchecks: lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS,MODEL,TRAN; df -hT; findmnt; git status --short; git branch --show-current; git rev-parse HEAD; git stash list; git ls-remote origin refs/heads/main refs/heads/research/phase-be-e0-e4-shadow. Do notwritefromunverifiedcwd orfall backtohome/tmp.

Officialmetadataqueries (notpayload): HEAD each exactKITTIURL inDATASET_FEASIBILITY; GET https://racecar-dataset.s3.us-west-2.amazonaws.com/?list-type=2&prefix=RACECAR-ROS2/S3/ and chosenmetadata.yaml. Metadata.xmlreportsbytes; S3multipartETagisnotfull-fileMD5. Publicmanifestkeepsremoteidentities. ExistingZIPdirectoryinventorywasreviewed, notareprocommandtodownloadarchivesbeforeapproval. DetailedprivatereceiptsincludeHTTPheaders/memberdirectory, officialreferenceGitHubtreeSHA and fileidentities. Do nothashallhistoricalinputs.

Afterapproval: verifydiskUUIDandfreespaceagain; configureTMPDIR/XDG_CACHE_HOME/PYTHONPYCACHEPREFIX/ROS_LOG_DIRunderworkroot; streamdownloadsintoexactapprovedpath, verifyexpectedsizeandnewarchiveSHA/ZIPintegrity; preservearchives; extractonlyapprovedsensors/calibration. Ifnames/size/access/peakchangesmateriallySTOPforreapproval. No commandsinthisreportautostartdownload.

Beforematching: passKITTIinputmapaudit, version/source/binary freeze, oldBEcertificatesnotreusedfornewdata. FreezeRcomparisonasCOVARIANCE_COMPARISON. Preserveallfailuresandactualsubmissiontimestamps. AllresultfilescurrentlyNOT_RUN; performanceandaccuracycannotbereproducedbecausenotyetmeasured. RACECARrequiresasecondrequestafterKITTIresults andfreshdiskcheck. NohistoricalcleanupauthorizedatBF0.


# BF1 reproducibility after the approved acquisition

Set BF_ROOT to the approved work root on verified newvolume; REPO to thisindependentcheckout; MATCHER to the authorizedexistingmatch_p0binary. Never substitutesystem/T7paths. Originalthreearchives, expectedlengths/allCRC/SHAidentities inBF1_MANIFEST. Select syncvelodyne_points/oxts, extractoxts andthreecalibrationtextfiles; donotextractimages. Keeparchives.

```bash
export TMPDIR="$BF_ROOT/tmp" XDG_CACHE_HOME="$BF_ROOT/tmp/cache" MPLCONFIGDIR="$BF_ROOT/tmp/mpl" CUDA_CACHE_PATH="$BF_ROOT/tmp/cuda-cache"
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$REPO/src" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4
# Set LD_LIBRARY_PATH to the private qualifieddependencydirectory documentedinlocalidentityledger.
python3 experiments/phase_bf_public_dataset_validation/kitti_adapter.py --raw "$BF_ROOT/kitti/raw" --out "$BF_ROOT/kitti/NEW_PREP"
python3 experiments/phase_bf_public_dataset_validation/run_kitti.py --root "$BF_ROOT/kitti/NEW_PREP" --out "$BF_ROOT/results/NEW_B0" --matcher "$MATCHER" --mode B0 --raw "$BF_ROOT/kitti/raw"
```

The original canonicaldatasetprep is derived_time_aligned; derived is a retainedinvalidtimestamp preparation, neveruseitformatching. Primarycontrol usesB0_run1arrival_schedule (57scan→availableepochs, generatedfromB0recordswithoutGT), andB1_fixed_arrival --arrival-schedule thatJSON. Actualcomputeprofilingkeptseparately, notassumedtomeet imposeddeadlines. CodeconfigsetsfixedGLL3/DIRECT7 andunchangedQ/Bias/domainrules; noexperimentgrid.

Offline only:

```bash
python3 experiments/phase_bf_public_dataset_validation/evaluate_kitti.py --root "$BF_ROOT/kitti/derived_time_aligned" --results "$BF_ROOT/results/kitti" --out "$BF_ROOT/results/kitti/NEW_EVAL" --b1-run B1_fixed_arrival
python3 -m pytest tests/test_bf_kitti_adapter.py tests/test_pose_uncertainty.py tests/test_external_pose.py tests/test_delayed_local.py tests/test_be_managed_gmf.py -q -p no:cacheprovider --basetemp="$BF_ROOT/tmp/NEW_TESTS"
```

Do not rerunintoexistingoutputs oroverwritefailedprefixes. Rawscans77, map0–19/test20–76; allfullstate/P, rawmatchparameters/nativeR, frozenK/proposals and actualcommits are inper-runframes.jsonl. Temporalreplay determinismrequires thesamearrival schedule; CUDAfloatingpoint/executiontimings canvary. Exactrebuildparityofprivatebackendisnotclaimed; binary/librarySHA mandatory. DownloadingRACECARrequires separateapprovalandfreshcapacitycheck.
