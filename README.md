# SLAM-visualizer
Rviz2 visulizer from input file
**TODO RENAME TO SLAM-VISUALIZER**.

-------------------------------------------------------------------------------

## Data
 - Example files are stored in `data` folder
 - Folder contains toy data
 - After updating data you have to rebuild

### Points
 - `points.csv` contains all landmarks
 - `ID` of the landmark is his line number (starting from zero)
 - `x.x x.x x.x` numbers separated by space represents 3D coordinates

### Positions
 - `positions.csv` contains all visited positions
 - `ID` of the position is his line number (starting from zero)
 - `x.x x.x x.x` numbers separated by space represents 3D coordinates

### Seen
 - `x y` two integers separated by space represents ID of the landmark (`y`)
   visible from the position with ID (`x`)

-------------------------------------------------------------------------------

## Rviz2 settings
 - `world` frame
 - `visualization` namespace
 - `viz` topic name

-------------------------------------------------------------------------------

## Scripts
 - `. scripts/build.sh` build repo
 - `. scripts/clean.sh` clean build files
 - `. scripts/run.sh` run visualization
