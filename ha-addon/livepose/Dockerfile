# Stock Vacuula 1.1.0 plus the live pose change: only the changed / new server modules and the rebuilt web app
# are replaced. build.sh puts them into overlay/ and web/ next to this file.
FROM ghcr.io/vacuula/vacuula:1.1.0

# the upstream image installs the server (non-editable) into its venv: /server/.venv/..., web app in /server/web
ARG SITE_PACKAGES=/server/.venv/lib/python3.14/site-packages

# The base must be the very 1.1.0 build the overlay was made against. base-sha256.txt holds the sha256 of every file
# the overlay replaces (8 server modules, web/index.html and its assets; same for arm64 and amd64). A retagged or
# rebuilt base image fails the build here instead of running a mix of two versions. livepose*.py are new files.
COPY base-sha256.txt /tmp/base-sha256.txt
RUN sha256sum -c --strict /tmp/base-sha256.txt \
    && test ! -e ${SITE_PACKAGES}/vacuula_server/livepose.py \
    && test ! -e ${SITE_PACKAGES}/vacuula_server/livepose_core.py \
    && rm /tmp/base-sha256.txt

COPY overlay/vacuula_server/ ${SITE_PACKAGES}/vacuula_server/

RUN rm -rf /server/web
COPY web/ /server/web/

# fail the build, not the first start: syntax errors, wrong install path, missing web app
RUN test -f ${SITE_PACKAGES}/vacuula_server/_build_version.py \
    && python -m compileall -q ${SITE_PACKAGES}/vacuula_server \
    && grep -q robot_pos ${SITE_PACKAGES}/vacuula_server/livepose.py \
    && test -f /server/web/index.html
