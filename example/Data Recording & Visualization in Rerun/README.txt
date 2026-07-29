# Recording and Visualizing Field Data in Rerun

This guide walks through recording a data run on the Booster robot, pulling it onto your laptop, and viewing it locally with [Rerun](https://www.rerun.io/).

## Prerequisites

- SSH access to the booster (`booster@192.168.50.1`)
- `rerun-cli` (or the `rerun` Python package) installed locally
- `plot_odometry.py` available in your working directory

## 1. Record data on the booster

Run the recording script:

```bash
./record_data.sh 
```

This records a new run into `~/k1_field_data/FIELD_XXX` on the robot (rosbag + associated files). Stop the recording when you're done (e.g. `Ctrl+C`), and note the run folder name (e.g. `FIELD_001`) for the next step.

## 2. Copy the data from the booster to your laptop

Pull the entire field-data folder for the run you want to inspect:

```bash
scp -r booster@192.168.50.1:~/k1_field_data/FIELD_001 ./
```

This copies the whole `FIELD_001` directory — including the raw rosbag — into your current directory.

## 3. Convert the rosbag to a Rerun recording (`.rrd`)

```bash
rerun mcap convert FIELD_001/rosbag/rosbag_0.mcap -o FIELD_001.rrd
```

This reads the MCAP-format rosbag and converts it into Rerun's native `.rrd` format, which loads much faster in the viewer than re-parsing the bag every time.

## 4. Add odometry plots to the recording

```bash
python3 plot_odometry.py FIELD_001/rosbag --rrd FIELD_001.rrd
```

This script reads odometry topics from the rosbag and logs them into the existing `FIELD_001.rrd` file as time-series plots (e.g. position, velocity, heading), so they show up alongside the rest of the recording in the viewer.

## 5. Open the recording in Rerun

```bash
rerun FIELD_001.rrd
```

This launches the Rerun Viewer with the full recording loaded — sensor data, odometry plots, and any other logged streams from the run.

---

## Notes

- Repeat steps 1–4 for each new `FIELD_XXX` folder you pull from the robot.
- If you only need to re-view an existing `.rrd` (no new data), you can skip straight to step 5.
- Keep `.rrd` files named after their source folder (`FIELD_001.rrd`, `FIELD_002.rrd`, ...) to avoid mixing up runs.
