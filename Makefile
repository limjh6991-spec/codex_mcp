.PHONY: quick egl_only gui_review headless_gate clean_headless_dumps

quick:
	CARB_APP_QUIET_SHUTDOWN=0 ROARM_DISABLE_HEADLESS_OVERLAY=1 \
	python open_roarm_m3_gui.py assets/roarm_m3/roarm_m3.usd --mode train --warmup-steps 10 --max-steps 120 --no-quiet-shutdown --no-blocklist

egl_only:
	__GLX_VENDOR_LIBRARY_NAME= \
	CARB_APP_QUIET_SHUTDOWN=0 ROARM_DISABLE_HEADLESS_OVERLAY=1 \
	python open_roarm_m3_gui.py assets/roarm_m3/roarm_m3.usd --mode train --max-steps 120 --no-quiet-shutdown --no-blocklist

gui_review:
	CARB_APP_QUIET_SHUTDOWN=0 python open_roarm_m3_gui.py assets/roarm_m3/roarm_m3.usd --mode review --max-steps 600 --no-quiet-shutdown

headless_gate:
	python scripts/check_headless_log.py logs/run_headless_hardblock.log

clean_headless_dumps:
	python scripts/cleanup_headless_dumps.py
