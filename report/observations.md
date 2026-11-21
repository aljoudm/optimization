# Observations for the report (numbers regenerate with `scripts/asset_zone_summary.m`)

## O1. The existing camera sites sit where the risk is lowest
Of the 37 existing camera sites that fall on the boundary (5 checkpoints + 32 interdiction points, positions ASSUMED by the
road-crossing rule), **24 are in Cold cells and only 2 in Hot cells** (11 in Neutral). Measured as covered boundary, the cameras
cover about **5.2 % of the Cold boundary, 1.9 % of the Neutral and 0.5 % of the Hot boundary** (2.6 % overall).
Mean risk of the cells that host a camera site is 0.217 against 0.344 for all cells.

Reason: the cell risk in DAS-map grows with distance from roads (and with slope), and the existing cameras are placed at road
crossings (vehicle inspection). So the present systems watch the easy-to-reach areas and the high-risk areas (wadis, mountains,
far from roads) are the least covered. This is the main argument for adding DAS along the boundary.

Caveats (to be stated in the report): camera positions are ILLUSTRATIVE (real ones are not public); one site covers one 100 m stretch
(ASSUMED footprint, all cameras of a site watch the same lanes); the count of cameras per interdiction point (4) is a PLACEHOLDER.
An earlier run with a different (wrong) match for Al-Sharaye gave 26 of 37 in Cold and 1 in Hot; the figures above are the corrected ones.

## O2. Checkpoint matching to OSM roads (checked against the OSM data itself)
* Al-Hada = Taif road via Al-Kara / Al-Hada, OSM ref 15, name "طريق الكرا الهدا": crosses the boundary on the east side.
* The OSM way "طريق الأمير سلطان / At Taif Road" (ref 15) is the SAME Taif-Hada road (its ways join the Hada way), so it cannot be Al-Sharaye.
* Route 40 continues east / north-east of Makkah (King Faisal Road -> "Makkah Al-Mukarramah Road", ref 40). Al-Sharaye is matched to its
  crossing with the boundary. OSM never uses the word "Sail": this identification rests on ref continuity and on the project lead's statement.
