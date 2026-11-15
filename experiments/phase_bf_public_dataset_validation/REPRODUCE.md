# BF0 reproduction and stop conditions

Scope: reproduce metadata review only, notdatasetexperiment. Researchcodebaseline da3ea9df0acf34eb5c0808c2e258c9bbb86bafb1. No configurationoradapterimplemented; NO_BF_REPLAY.

Read-onlyenvironmentchecks: lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINTS,MODEL,TRAN; df -hT; findmnt; git status --short; git branch --show-current; git rev-parse HEAD; git stash list; git ls-remote origin refs/heads/main refs/heads/research/phase-be-e0-e4-shadow. Do notwritefromunverifiedcwd orfall backtohome/tmp.

Officialmetadataqueries (notpayload): HEAD each exactKITTIURL inDATASET_FEASIBILITY; GET https://racecar-dataset.s3.us-west-2.amazonaws.com/?list-type=2&prefix=RACECAR-ROS2/S3/ and chosenmetadata.yaml. Metadata.xmlreportsbytes; S3multipartETagisnotfull-fileMD5. Publicmanifestkeepsremoteidentities. ExistingZIPdirectoryinventorywasreviewed, notareprocommandtodownloadarchivesbeforeapproval. DetailedprivatereceiptsincludeHTTPheaders/memberdirectory, officialreferenceGitHubtreeSHA and fileidentities. Do nothashallhistoricalinputs.

Afterapproval: verifydiskUUIDandfreespaceagain; configureTMPDIR/XDG_CACHE_HOME/PYTHONPYCACHEPREFIX/ROS_LOG_DIRunderworkroot; streamdownloadsintoexactapprovedpath, verifyexpectedsizeandnewarchiveSHA/ZIPintegrity; preservearchives; extractonlyapprovedsensors/calibration. Ifnames/size/access/peakchangesmateriallySTOPforreapproval. No commandsinthisreportautostartdownload.

Beforematching: passKITTIinputmapaudit, version/source/binary freeze, oldBEcertificatesnotreusedfornewdata. FreezeRcomparisonasCOVARIANCE_COMPARISON. Preserveallfailuresandactualsubmissiontimestamps. AllresultfilescurrentlyNOT_RUN; performanceandaccuracycannotbereproducedbecausenotyetmeasured. RACECARrequiresasecondrequestafterKITTIresults andfreshdiskcheck. NohistoricalcleanupauthorizedatBF0.
