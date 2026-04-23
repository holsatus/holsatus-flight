# Landing log

Manual ground-truth record of each flight's landing position, paired
with the firmware's flow-integrated displacement estimate for calibration.

## How to fill a row

After each flight:

1. **Tape-measure the distance** from the chalk/tape "takeoff" mark to
   the centre of the drone's final landing position. Record in metres.
2. **Record the direction** as a short free-text ("toward cupboard",
   "toward window", or cardinal "+x" / "-y" if you set an axis convention).
3. **Copy `|d|`** from the log's last `[mission] flow_disp ...` line
   (appears right before `[mission] disarming`).
4. **Copy the git sha** from the log's `[free] git=...` line at the top.
5. **Copy battery mV** from `[bat] voltage=...` near the top.
6. **Notes**: wobbled, soft/hard landing, any drift direction change,
   which bin the drone landed in, etc.

The point of this log is comparing columns `tape_m` and `flow_|d|_m`
over several flights:

- If they agree within ~20 % across runs, `FLOW_SCALE = 0.25` in
  `mtf01_reader_task` is trustworthy as a short-duration displacement
  estimator. Good enough for autonomous position hold experiments.
- If `flow_|d|_m` is systematically smaller than `tape_m`, bump
  `FLOW_SCALE` up proportionally and re-measure.
- If they disagree inconsistently (sometimes matches, sometimes way off),
  the flow sensor is losing quality during the flight and we need to
  look at `flow_q` mid-flight to see when it drops out.

## Log

| D-num  | git_sha    | bat_mV | tape_m | tape_dir (degrees)         | flow_\|d\|_m | notes                              |
|--------|------------|--------|--------|------------------|--------------|------------------------------------|
| D00079 | 4fd42bac5e | 15852  | 0.68        | 130                 |              |    some wobble decent flight                                 |
| D00081 | 4fd42bac5e | 15787  | 0.48        | 100                 |              |    some wobble decent flight                                 |
| D00082 | 4fd42bac5e | 15836  | 0.34        | 130                 |              |    only reached .4-.5 m altitude                                |
| D00083 | e5488407fc | 16446  | 0.96        | 120                 |              |    reached full altitude, solid flight                                |
| D00084 | e5488407fc | 16325  | 1.86        | 120                 |              |    drift further and longer than usual                                |
| D00085 | e5488407fc | 16139  | 1.02        | 95                 |              |    reached full altitude, small uplift before shut-off                                |
| D00086 | e5488407fc | 16288  | 1.35        | 130                 |              |    raised to about 1.3-1.4 m, small yank to the left before landing, off newspaper                                |
| D00087 | 79997c8ea4 | 16149  | 1.55        | 130                 |              |    small jump at end and crash                                |
| D00088 | 79997c8ea4 | 16077  | 1.55        | 130                 |              |    small jump at end and crash                                |
| D00090 | 79997c8ea4 | 15947  | 0.55        | 130                 |              |    small jump at end and crash                                |
| D00093 | 042250a462 | 15788  | 1.25        | 120                 | N/A          |    longer hover before end; log cut off mid-descent       |
| D00094 | 042250a462 | 15729  | 1.67        | 140                 | N/A          |    bounce at 0.18m; log cut off mid-descent               |
| D00095 | 042250a462 | 15731  | 1.38        | 140                 | N/A          |    bounce at 0.18m; log cut off mid-descent               |
| D00099 | 13270c2727 | 16427  | 0.49        | 92                 | N/A          |    still little bounce at the end               |
| D00101 | 13270c2727 | 16356  | 1.26        | 93                 | N/A          |    still little bounce at the end               |
| D00102 | 13270c2727 | 16279  | 1.07        | 45                 | N/A          |    still little bounce at the end               |
| D00104 | 13270c2727 | 16255  | 2.14        | 100                 | N/A          |   bounce near the end, extended hover near ground, soft crash               |
| D00105 | 13270c2727 | 16176  | 0.70        | 110                 | N/A          |    despite not so far drift, pronounced bounce at the end and hard crash, broke 1 propeller               |
