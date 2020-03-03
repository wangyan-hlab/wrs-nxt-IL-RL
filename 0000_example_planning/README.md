# The fundamental steps to build a RDB supported regrasp planner
# Author: Weiwei, Date: 2019-03-23

## Structure of the package:

The packages include:
1. obstacles
    * A folder that hosts the stl models of the environment (including robot bodies (not robot arms))

2. objects
    * A folder that hosts the stl models of target objects

4. freegrip.py, freetableplacement.py, tabletopplacements.py, floatingposes.py
    * These files create the database for freegrabs, table-top placements at (0,0,0), 
    table-top placements at discretized positions, floating poses, as well as their associated grabs

5. ur3drrt.py, ur3drrtct.py, ur3drrthold.py motion
    * Motion planning examples. ur3drrt.py: Simple joint space motion planning using rrtconnect; 
    ur3drrtct.py: Joint space motion planning considering constraints. This file demonstrates how 
    to add a constraint function "ctchecker" to the ctcallback; ur3drrthold.py: Motion planning considering
    collisions between objects and environment.
   
6. gui_teach.py
    * The teaching GUI

## Executing the the teaching GUI

To execute the teaching GUI you need to build a database

1. Create your database in MySQL
    * Run createdb.sql in MySQL to create the database
    
2. Compute the free grasps using freegrip.py
    * Free grasps are the planned grasps without considering any surrounding obstacles

3. Compute the free stable placements using freetableplacement.py
    * Stable placements are the stable poses of an object laying on a table
    * Free tableplacements means we do not consider about translation of the object (it is set to 0,0,0 by default)

4. Compute the stable placements and ik-feasible grabs all over a table surface using tabletopplacements.py
    * Translations and Ik of robot are further considered

5. Compute the floating poses and ik-feasible grabs of the floating poses using floatingposes.py
    * Handover states

6. Plan motion sequences using main.py

## To make changes

In the program, you can change the database at any step in ”Executing the the teaching GUI 1-5“. The relational constraints will automatically help to
maintain the integrity of the data. Thus, step 1 in this section is no longer needed.
Nevertheless, you can still run it if you dont mind deleting the whole database and startover.

1. Resetting the database.

    1.1 Use the following SQL commands in MySQL workbench to clear the DB.

    Delete the grasp data (in MySQL workbench):
    ```sql
    SET FOREIGN_KEY_CHECKS=0;
    truncate table ur3dualgrip.tabletopplacements;
    truncate table ur3dualgrip.tabletopgrips;
    truncate table ur3dualgrip.ik;
    truncate table ur3dualgrip.freeairgrip;
    truncate table ur3dualgrip.freetabletopplacement;
    truncate table ur3dualgrip.freetabletopgrip;
    SET FOREIGN_KEY_CHECKS=1;
    ```

    Delete the tabletop regrasp data (in MySQL workbench):
    ```sql
    SET FOREIGN_KEY_CHECKS=0;
    truncate table ur3dualgrip.tabletopplacements;
    truncate table ur3dualgrip.tabletopgrips;
    truncate table ur3dualgrip.ik;
    SET FOREIGN_KEY_CHECKS=1;
    ```

    Delete the floating pose data (in MySQL workbench):
    ```sql
    SET FOREIGN_KEY_CHECKS=0;
    truncate table ur3dualgrip.floatingposes;
    truncate table ur3dualgrip.floatinggrips;
    truncate table ur3dualgrip.floatinggripspairs;
    truncate table ur3dualgrip.ikfloatinggrips;
    SET FOREIGN_KEY_CHECKS=1;
    ```

    Delete the dual-arm assembly data (in MySQL workbench):
    ```sql
    SET FOREIGN_KEY_CHECKS=0;
    truncate table ur3dualgrip.assembly;
    truncate table ur3dualgrip.assemblyx;
    truncate table ur3dualgrip.assemblyxgrippairs;
    truncate table ur3dualgrip.assemblyxgrips0;
    truncate table ur3dualgrip.assemblyxgrips1;
    truncate table ur3dualgrip.ikassemblyxgrips0;
    truncate table ur3dualgrip.ikassemblyxgrips1;
    SET FOREIGN_KEY_CHECKS=1;
    ```

    1.2 Update table height

    Delete the tabletop regrasp data (in MySQL workbench):
    ```sql
    SET FOREIGN_KEY_CHECKS=0;
    truncate table ur3dualgrip.tabletopplacements;
    truncate table ur3dualgrip.tabletopgrips;
    truncate table ur3dualgrip.ik;
    SET FOREIGN_KEY_CHECKS=1;
    ```

    Change the following code in tableplacements.py (manipulation/regrasp) to increase 
    the number of stable placements on tables
    ```
    grids = []
    for x in range(400,551,100):
        for y in range(-300,301,600):
            grids.append([x,y,1084])
    ```
    The 0 in the last row, namely [x,y,0], indicates table heights
    The default table height is 1084

    1.3 Dont forget to replace the models for collision detection

    Revise
    ```
    regrip = regriptppfp.RegripTppFp(objpath, hrp5nrobot, handpkg, gdb, base, obstaclelist)
    ```
    in hrp5nmixplot_withcd.py (manipulation/regrasp/examples)

    1.4 Rerun E2 to recompute the tabletopplacements. Rerun E5 to generate motion sequence.
