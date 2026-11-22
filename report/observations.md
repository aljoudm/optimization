# Observations for the report (numbers regenerate with `scripts/asset_zone_summary.m`)

## O1. The existing camera sites sit where the risk is lowest
With the 32-site case (5 checkpoints + 22 real interdiction crossings + 10 filler sites, positions ASSUMED by the road-crossing rule), 37 camera sites fall on the boundary:
**25 in Cold cells, 10 in Neutral and only 2 in Hot cells.** With the 22 real crossings only (27 sites) the split is 21 Cold, 6 Neutral and 0 Hot.
The cameras cover about 5.4 % of the Cold boundary, 1.7 % of the Neutral and 0.5 % of the Hot boundary (2.6 % overall, 32-site case); with the 22 real crossings 4.6 %, 1.0 %, 0.0 % (1.9 % overall).
Mean risk of the cells that host a camera site is 0.212, against 0.344 for all cells.

Reason: the cell risk in DAS-map grows with distance from roads (and with slope), and the existing cameras are placed at road crossings (vehicle inspection). So the present systems watch the easy-to-reach areas and the
high-risk areas (wadis, mountains, far from roads) are the least covered. This is the main argument for adding DAS along the boundary.

Caveats: camera positions are ILLUSTRATIVE (real ones are not public); one site covers one 100 m stretch (ASSUMED footprint; all cameras of a site watch the same lanes); the number of cameras per interdiction point (4) is a PLACEHOLDER.
(Earlier drafts of this note, with a wrong match for Al-Sharaye or before the interdiction-point clearance fix, gave 26 / 24 Cold and 1 / 2 Hot; the figures above replace them.)

## O2. Checkpoint matching to OSM roads (checked against the OSM data itself)
* Al-Hada = Taif road via Al-Kara / Al-Hada, OSM ref 15, name "طريق الكرا الهدا": crosses the boundary on the east side.
* The OSM way "طريق الأمير سلطان / At Taif Road" (ref 15) is the SAME Taif-Hada road (its ways join the Hada way), so it cannot be Al-Sharaye.
* Route 40 continues east / north-east of Makkah (King Faisal Road -> "Makkah Al-Mukarramah Road", ref 40). Al-Sharaye is matched to its crossing with the boundary. OSM never uses the word "Sail": this identification rests on ref continuity and on the project lead's statement.
