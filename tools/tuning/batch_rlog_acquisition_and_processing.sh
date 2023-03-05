#!/usr/bin/env/bash

/usr/bin/env/bash /mnt/video/scratch-video/rlog_api/rlogs_copy_from_device_to_local_folder.sh

/usr/bin/env/bash /mnt/video/scratch-video/rlog_api/api1.sh

/usr/bin/env python3 tools/tuning/sort_rlogs.py

/usr/bin/env/bash tools/tuning/batch-rlog-to-lat.sh