# Shell Racer backup and rebuild

This repository contains a public source export of the four live Shell Racer flow tabs, the latest Launch Boost firmware with Wi-Fi and server placeholders, and the iPad guide. The starting flow revision is recorded in `node-red/revision.txt`.

## Set up the backup

1. Review the public source export for local infrastructure details and credentials before publishing changes. The repository already contains the starting source snapshot.
2. On the Node-RED host, set up a repository-specific SSH deploy key with write access, then clone the repository to a dedicated folder. Keep the private key on the host. Use a separate clone for human edits.
3. Copy `backup/config.example.json` to `backup/config.json` in the host checkout. Set `checkout` and `flows_file` to the actual absolute paths. Find the flow and settings paths in the Node-RED startup log. If using Docker, point to the host-mounted paths and run the job as a user with read access. Use `firmware_sources` for the current Arduino sketch files when those changes are made on the host. Each sketch must use a same-named parent folder. For firmware edited elsewhere, commit the sanitised sketch from that machine.
4. Set a Git commit identity for the checkout. Run `python3 backup/snapshot.py backup/config.json` from it. Check that the new commit is visible on GitHub.
5. Run the same command every 15 minutes under the host account using cron or its existing scheduler. Use absolute paths. For cron, substitute your actual paths in this example.

```cron
*/15 * * * * /usr/bin/python3 /path/to/checkout/backup/snapshot.py /path/to/checkout/backup/config.json >> /path/to/shell-racer-backup.log 2>&1
```

The source job commits only when the tracked export or configured firmware changes. Review the backup log and confirm the remote has a recent commit after making a known change. Keep the checkout clean for scheduled runs.

## Back up the state needed for a complete rebuild

Keep server recovery archives, audio, and race results in a separate private destination. This public checkout runs source snapshots only.

The flow export and firmware alone cannot recover the MP3 library, saved race results, colour calibration or server configuration. In the private rebuild kit, list the approved recovery paths in `recovery_paths`, configure an `age` public recipient and run the recovery job from your separate private rebuild kit when important state changes or on a nightly schedule. Install `age` on the host first. Keep the matching private identity in your password manager or other off-server recovery location.

- Find the actual Node-RED user directory and service or container configuration. For a dedicated Shell Racer instance, include its persistent user data and settings. Review that scope before uploading a shared instance. Include the relevant context store, flows and credentials files, runtime deployment configuration and any external credential secret needed to restore them. Store separately managed credentials in your password manager. Avoid including `node_modules`, cache and logs.
- Include the audio directory if it exists. The flow reads `SHELLRACER_AUDIO_DIR`, with a local default in its audio node. Uploaded audio and assignments can also live in the persistent Node-RED context. Include the palette package manifest and lockfile where used.
- The private recovery script encrypts the recovery archive before writing it to disk, then pushes the ciphertext to its separate private destination. Recovery archives in Git are limited to 90 MiB per archive and each nightly copy adds to Git history. For larger archives use a separate encrypted off-host backup destination.
- Run a restore drill. Save the exact Node.js, Node-RED and dashboard palette versions, the service name or Docker image, and the original path for each recovery item. Check the archive can be decrypted from a separate machine. If copying a running instance, schedule recovery during idle time and verify the snapshot.

## Rebuild on a replacement server

1. Clone this repository or the separate private recovery repository. Install the original Node-RED and dashboard dependencies. The export uses FlowFuse Dashboard 2 widgets. If restoring a full archived instance, use the saved dependency manifest and lockfile.
2. With Node-RED stopped, decrypt the recovery archive into a staging directory. Inspect the contents, then restore them to the appropriate runtime paths with the Node-RED service account ownership. Restore the original credential secret where required. Keep the node IDs and context data together. Do not import a second copy of the flows if the full runtime files have already been restored.
3. For a source-only rebuild, import `node-red/ShellRacer.flows.json` into a fresh instance. Recreate the `shellRacerRam` memory and `shellRacerDisk` local filesystem stores in Node-RED settings before starting. Set `SHELLRACER_AUDIO_DIR` to the audio folder and, if needed, `SHELLRACER_ALLOWED_HOSTS` to a comma-separated list of additional trusted dashboard hostnames. Restore track calibration and review Fleet settings.
4. Point your existing server hostname at the new host where practical. If reflashing a car, fill in the Wi-Fi settings, Node-RED URL and that car's `CAR_NAME` in a local copy of the sketch.
5. Verify ShellRacer Live, Race, Drive, Simulator and Results. Check one car for telemetry and control before running a race. Test one blue-strip crossing, audio and saved results.

Never commit plain-text Wi-Fi passwords, Node-RED credentials, SSH private keys or unencrypted recovery archives.
