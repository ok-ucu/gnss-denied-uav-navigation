# План фінального staging — лише для перевірки

Git index не змінено: **git add / commit / push не виконувалися**.
Гілка — `mavros-integration`. Перегляньте diff перед окремим дозволом на staging.

[Integration README](README.md) · [SITL](SITL.md) ·
[Живий звіт](experiments/ekf444_covariance_20261104/REPORT.md) ·
[Явний список файлів](STAGING_FILES.txt) ·
[Manifest доказів](experiments/ekf444_covariance_20261104/SHA256SUMS.json).

## Обсяг запропонованого commit

Обрано **36 файлів**: 1 tracked-документ (`integration/README.md`) зі змінами
та 35 нові файли. Код інших учасників і наявні робочі інструменти не змінено.
Розмір нижче — сума повних поточних файлів, а не compressed Git pack або
розмір текстового diff.

- Разом: **33,779,448 bytes (32.21 MiB)**.
- Канонічний BIN: **22,298,624 bytes (21.27 MiB)**.
- Канонічний TLOG: **3,553,595 bytes (3.39 MiB)**.
- Первинний received.jsonl: **6,468,199 bytes (6.17 MiB)**.

BIN і capture включено навмисно: без первинних логів незалежний аналіз
звітних цифр із чистого clone був би неможливим. Git LFS не налаштовувався,
первинні файли не стискалися, не перекодовувалися й не змінювалися.
Локальний [`.gitattributes`](.gitattributes) задає `-text` для доказів,
щоб майбутній git add не нормалізував CRLF і не порушив SHA-256.
`whitespace=cr-at-eol` для CSV враховує штатні CSV CRLF, не змінюючи байтів.
Для verbatim source_evidence.txt та docker.log збережено пробіли в порожніх
рядках після префіксів номерів/контейнера: тільки ці два evidence-файли мають
`whitespace=-blank-at-eol`. Код і документація такого винятку не мають.

## Включити — точний список від кореня репозиторію

Список у наступному блоці тотожний `STAGING_FILES.txt`. Це список для
майбутнього явного staging, **не виконана команда**. Не використовуйте
`git add integration/`.

```text
integration/.gitattributes
integration/.gitignore
integration/README.md
integration/SITL.md
integration/STAGING.md
integration/STAGING_FILES.txt
integration/analyze_live_ekf.py
integration/run_live_ekf.py
integration/sitl.compose.yaml
integration/sitl_ext_nav.parm
integration/sitl_transport.parm
integration/start_sitl.sh
integration/verify_sitl.py
integration/sitl_grounded_smoke.csv
integration/test_ekf_covariance.py
integration/test_ekf_covariance.md
integration/tests/test_covariance_experiment.py
integration/tests/test_live_ekf_analysis.py
integration/tests/test_sitl_setup.py
integration/experiments/ekf444_covariance_20261104/generator.csv
integration/experiments/ekf444_covariance_20261104/summary.json
integration/experiments/ekf444_covariance_20261104/parameters.json
integration/experiments/ekf444_covariance_20261104/synthetic_measurements.csv
integration/experiments/ekf444_covariance_20261104/REPORT.md
integration/experiments/ekf444_covariance_20261104/warmup.csv
integration/experiments/ekf444_covariance_20261104/monitor.csv
integration/experiments/ekf444_covariance_20261104/fusion_logic_source.txt
integration/experiments/ekf444_covariance_20261104/status_bits_source.txt
integration/experiments/ekf444_covariance_20261104/source_evidence.txt
integration/experiments/ekf444_covariance_20261104/received.jsonl
integration/experiments/ekf444_covariance_20261104/docker.log
integration/sitl_runs/20261104T001121Z-7/logs/00000001.BIN
integration/sitl_runs/20261104T001121Z-7/mav.tlog
integration/sitl_runs/20261104T001121Z-7/firmware-commit.txt
integration/sitl_runs/20261104T001121Z-7/architecture.txt
integration/experiments/ekf444_covariance_20261104/SHA256SUMS.json
```

## Не включати — наявні локальні файли

Ці файли збережені на диску. Новий `integration/.gitignore` виключає їх;
жоден із них не видалено. Також не включайте кореневий `.DS_Store`,
локальні virtualenv, Python/editor caches та наступні непогоджені runs.

```text
integration/.baseline.json
integration/Dockerfile.test
integration/experiments/ekf444_covariance_20261104/analysis_stdout.json
integration/experiments/ekf444_covariance_20261104/dataflash_selected.csv
integration/gps_track.csv
integration/sitl_connectivity.json
integration/sitl_generator_smoke.csv
integration/sitl_monitor_smoke.csv
integration/sitl_runs/20261104T000426Z-7/architecture.txt
integration/sitl_runs/20261104T000426Z-7/defaults.parm
integration/sitl_runs/20261104T000426Z-7/eeprom.bin
integration/sitl_runs/20261104T000426Z-7/firmware-commit.txt
integration/sitl_runs/20261104T000426Z-7/image-source-path.txt
integration/sitl_runs/20261104T000426Z-7/mav.parm
integration/sitl_runs/20261104T000426Z-7/mav.tlog
integration/sitl_runs/20261104T000426Z-7/mav.tlog.raw
integration/sitl_runs/20261104T000426Z-7/terrain/S36E149.DAT
integration/sitl_runs/20261104T000526Z-7/architecture.txt
integration/sitl_runs/20261104T000526Z-7/defaults.parm
integration/sitl_runs/20261104T000526Z-7/eeprom.bin
integration/sitl_runs/20261104T000526Z-7/firmware-commit.txt
integration/sitl_runs/20261104T000526Z-7/image-source-path.txt
integration/sitl_runs/20261104T000526Z-7/mav.parm
integration/sitl_runs/20261104T000526Z-7/mav.tlog
integration/sitl_runs/20261104T000526Z-7/mav.tlog.raw
integration/sitl_runs/20261104T000526Z-7/terrain/S36E149.DAT
integration/sitl_runs/20261104T001121Z-7/defaults.parm
integration/sitl_runs/20261104T001121Z-7/eeprom.bin
integration/sitl_runs/20261104T001121Z-7/image-source-path.txt
integration/sitl_runs/20261104T001121Z-7/logs/LASTLOG.TXT
integration/sitl_runs/20261104T001121Z-7/mav.parm
integration/sitl_runs/20261104T001121Z-7/mav.tlog.raw
integration/sitl_runs/20261104T001121Z-7/terrain/S36E149.DAT
integration/sitl_runs/launch.log
integration/sitl_verify.json
```

## Уже є в origin, нових змін немає

Ці файли залишаються частиною репозиторію й потрібні для запуску, але не
потребують повторного staging у цьому commit:

```text
integration/analyze_trajectories.py
integration/check_integration.py
integration/mavros.launch.xml
integration/mavros_plugins.yaml
integration/tests/test_adapter.py
integration/tests/test_offline.py
integration/vio_adapter.py
```

## Політика доказів і SHA-256

Перед редагуванням manifest усі 16 початкових записів перевірено за файлами.
Фінальний manifest зберігає **14 старих значень без жодної зміни**.
Прибрано тільки записи `dataflash_selected.csv` (відтворюваний decode BIN)
та `analysis_stdout.json` (дублікат summary.json). Обидва файли залишено
локально; їх можна відтворити командою з REPORT.md. Додано SHA-256 наявних
`firmware-commit.txt` і `architecture.txt` із канонічного run. Разом у
manifest 16 файлів; manifest не хешує сам себе.

`REPORT.md` і первинні логи не редагувалися. Їхні старі SHA-256 не
перераховувалися для маскування змін. Report згадує також локальні похідні
файли та TLOG `.raw`: вони не входять до curated commit; BIN/TLOG і
primary received.jsonl збережено, декодовані таблиці можна отримати повторно.
Manifest є точним переліком включених доказів; STAGING_FILES.txt також
містить код, документацію та конфігурацію.

## Перевірки для reviewer

- У `.venv` із pymavlink: 39/39 офлайн-тестів; без pymavlink — 38 passed,
  1 skipped wire serialization test. Живий SITL під час цієї підготовки
  не запускався; підтверджений попередній запуск описує REPORT.md.
- Відносні Markdown links ведуть до файлів, що є в origin або в staging plan.
  Абсолютні локальні посилання для GitHub не використовуються.
- Усі 16 SHA-256 у manifest мають збігатися; усі evidence-файли включені.
- Кожен staging candidate існує й не ігнорується; локальні виключення
  ігноруються. Перевіряється також, що незапланованих untracked-файлів,
  доступних для staging усередині integration, немає.
- `git diff --check` застосовується до tracked README; нові текстові файли
  перевіряються окремо, бо до git add звичайний diff їх не бачить.
- Git index має залишатися незмінним до дозволу користувача.
