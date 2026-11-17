These three files are the stock `config/euroc_mav/` configuration of OpenVINS v2.7
(https://github.com/rpng/open_vins, commit 93adc24, GPLv3), copied unchanged.

`run_baseline.py` takes `euroc_estimator_config.yaml` as the template, overrides the keys it
needs (mono camera, initialisation, noise, anything passed with `--set`) and writes the two
`kalibr_*` files itself from `camera_params.json`. The two stock `kalibr_*` files are kept only
as a reference for the expected layout.
