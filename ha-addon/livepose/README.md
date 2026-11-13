# Vacuula (live pose): local add-on

Stock Vacuula 1.1.0 plus the **live robot position**: a patched Neato D7 sends its pose as UDP "NEAP v4"
datagrams while it cleans; the server turns them into a `robot_pos` event on the robot's `/connect` stream and
`GET /api/robot/<serial>/pose`, and the Map page draws the trail and the robot on top of the map.

This is a **local** add-on: the supervisor builds it on the HA host from this folder
(`FROM ghcr.io/vacuula/vacuula:1.1.0`, then only the changed server modules and the rebuilt web app on top).
The deploy steps below are meant to be run by hand, with the robot docked and idle.

## 1. Build the add-on folder (on the dev machine, changes nothing else)

```sh
cd ha-addon/livepose
nix-shell -p nodejs pnpm --run ./build.sh          # -> out/vacuula_livepose/
```

`out/vacuula_livepose/` holds `config.yaml`, `Dockerfile`, `base-sha256.txt`, `overlay/vacuula_server/` (the 10 server
files this change adds or modifies, listed in `build.sh`), `web/` (the rebuilt app), icons, and this README.


The `Dockerfile` checks the sha256 of every upstream file it replaces (`base-sha256.txt`) before overlaying, so a
retagged `ghcr.io/vacuula/vacuula:1.1.0` fails the build instead of mixing versions. If upstream really republishes
the tag, regenerate the list from the new image and re-check the overlay against it.

## 2. Settings

| option | default | meaning |
|---|---|---|
| `POSE_UDP_PORT` | `7700` here (`0` = off, also the default in the stock add-ons) | UDP port the robot sends to. If it is busy the server retries every 30 s (it logs once when it cannot bind, once when it succeeds). |
| `POSE_FORWARD` | `127.0.0.1:7701` | `host:port`: every raw datagram is copied there, best effort (for example to feed a separate recorder). Empty = off. A target that is this server's own pose port is refused (endless loop) and forwarding stays off. |
| `POSE_SERIAL` | empty | Robot to credit when the sender IP matches no connected Gen3 robot. Empty: the only known Gen3 robot is used. |

The other options are the stock ones (`VACUULA_PORT` defaults to 8089; check
`ha addons info <stock add-on slug>` and copy any value that differs).

Security: the pose port is plain UDP with no authentication, so any host on the LAN can inject poses, and with
`POSE_SERIAL` set every source IP is accepted. That is fine on a home LAN; do not expose the port beyond it.

Calibration (robot map frame -> pixels of a persistent map) comes from `livepose.json` in the data dir
(`/addon_configs/<this add-on>/livepose.json`, re-read when it changes):

```json
{"maps": {"2020-01-01T00:00:00Z": {"robot_to_px": [[24.1, 0.0, 260.0], [0.0, -24.1, 130.0]]}}}
```

`col = a*x + b*y + c`, `row = d*x + e*y + f` (x, y in metres). **A `livepose.json` is required for the robot to be drawn.** Without the file (or for a map id it does not list)
no transform is known and the Map page shows no robot. The map id is the persistent map's id as listed by
`GET /api/robot/<serial>/maps`. To fit a map, take a few points where the robot position is known (the dock, for
example), read the robot-frame pose from `/api/robot/<serial>/pose` and the matching pixel on the map image, and
solve for the six values. For a map at about 24 px per metre with the robot origin at pixel (260, 130) the entry is
the one above. If the robot is re-explored (new persistent map) the transform must be re-fitted.

## 3. Deploy (not done yet)

Do this only while the robot is **docked and idle**: not cleaning, no map upload running. A restart during an
upload loses the map. The stock add-on (`<stock add-on slug>`) is never modified, moved or uninstalled.

1. **Snapshot the database** (sqlite backup API, consistent while the stock add-on runs). It holds the robot's
   credentials, keep it on the host only:

   ```sh
   ssh root@<ha-host> 'mkdir -p /share/vacuula-backups && python3 - <<EOF
   import sqlite3, time
   src = sqlite3.connect("file:/addon_configs/<stock add-on slug>/vacuula.db?mode=ro", uri=True)
   dst = sqlite3.connect("/share/vacuula-backups/vacuula.db.pre-livepose-" + time.strftime("%Y%m%d-%H%M"))
   src.backup(dst)
   print("ok")
   EOF
   chmod 600 /share/vacuula-backups/vacuula.db.pre-livepose-*'
   ```

2. **Copy the folder to the host and install the local add-on** (builds on the HA host; needs to pull the stock image,
   which is cached already):

   ```sh
   scp -r out/vacuula_livepose root@<ha-host>:/addons/vacuula_livepose
   ```

   HA: Settings -> Add-ons -> Add-on store -> menu -> *Check for updates* (or reload), then *Local add-ons* ->
   **Vacuula (live pose)** -> Install. Do not start it yet. Its data dir is `/addon_configs/local_vacuula_livepose`
   (confirm with `ls /addon_configs`).

3. **Free the pose port**: if another process already receives the robot's UDP datagrams on 7700, stop it, or
   move it to 7701 so it receives the add-on's `POSE_FORWARD` copy instead. The add-on retries the bind every 30 s,
   so the order is not critical, but until 7700 is free no pose arrives.

4. **Stop the stock add-on** (Settings -> Add-ons -> Vacuula -> Stop) and switch its *Start on boot* off, so a host
   reboot cannot start both (same ports: 8089, 335, 3443). Leave it installed.

5. **Copy, never move, the data directory** into the local add-on's, and verify it. The script refuses to run if the
   target is not empty: a first start by mistake leaves its own `vacuula.db` and `-wal`/`-shm` there, and SQLite could
   replay that stale WAL onto the copied database. If it refuses, stop the local add-on, move the leftovers aside by
   hand, and run it again. The stock add-on must be stopped (step 4).

   ```sh
   ssh root@<ha-host> 'sh -s' <<'EOF'
   set -e
   SRC=/addon_configs/<stock add-on slug>
   DST=/addon_configs/local_vacuula_livepose
   SNAP=$(ls -t /share/vacuula-backups/vacuula.db.pre-livepose-* | head -1)          # from step 1
   [ -d "$DST" ] || { echo "no $DST: install the local add-on first (do not start it)"; exit 1; }
   [ -z "$(ls -A "$DST")" ] || { echo "$DST is not empty, not copying"; exit 1; }
   cp -a "$SRC"/. "$DST"/
   # the robot trusts the certificate in ssl/: every file must be byte-identical
   (cd "$SRC" && find ssl -type f) | while read -r f; do cmp "$SRC/$f" "$DST/$f"; done
   # the database: same content as the snapshot taken before the swap
   [ "$(sqlite3 "$DST/vacuula.db" .dump | sha256sum)" = "$(sqlite3 "$SNAP" .dump | sha256sum)" ] \
     || { echo "copied database differs from the snapshot"; exit 1; }
   echo "copy verified"
   EOF
   ```

   (Add a `livepose.json` to the copy; without one no robot is drawn, see section 2.) The robot keeps
   its credentials, runs, maps and boundaries: no re-pairing.

6. **Options + start**: in the local add-on's Configuration tab make the options equal to the stock instance's
   (check `VACUULA_PORT`), leave `POSE_UDP_PORT: 7700`, `POSE_FORWARD: "127.0.0.1:7701"`, `POSE_SERIAL: ""`, Save,
   Start, and switch *Start on boot* on. The log should show `Live pose listening on udp://0.0.0.0:7700` and, within
   a minute, `Robot <serial> connected`.

7. **Verify**: `http://<ha-host>:8089/api/robots` shows the robot connected;
   `/api/robot/<serial>/pose` answers `"status": "none"` until a clean starts, then `localizing` and
   `pose`; open the Map page. The first reset (docking) logs a dock check
   (`reset N px from the dock ... calibration ok`; a warning above 8 px means calibration drift or a lifted robot).

Notes:

- A newer stock add-on version may migrate **its own** database forward. That is fine (separate directory), but
  never point the fork at a migrated database: it is built for the 1.1.0 schema and refuses to start on a newer one.
- Runs, maps and boundaries created while the fork is running exist only in `/addon_configs/local_vacuula_livepose`.

## 4. Revert

1. Stop **Vacuula (live pose)** (and switch its *Start on boot* off).
2. Start the stock **Vacuula** add-on and switch its *Start on boot* back on. Its directory
   `/addon_configs/<stock add-on slug>` was never touched, the robot reconnects on its own.
3. If you moved another UDP receiver to a different port in step 3, put it back on 7700.
4. Optional: uninstall the local add-on and delete `/addons/vacuula_livepose` and
   `/addon_configs/local_vacuula_livepose` (it contains the DB copy and its credentials). The pre-swap snapshot stays
   in `/share/vacuula-backups/` until you delete it.

If something from the fork's run should be kept (new runs, a changed map), copy it over from
`/addon_configs/local_vacuula_livepose/` before deleting that directory. Do not copy `vacuula.db` over the stock
one while the stock add-on runs.
