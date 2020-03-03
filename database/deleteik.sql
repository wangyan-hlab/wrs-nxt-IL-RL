delete from ik using ik, tabletopgrips, tabletopplacements, freetabletopplacement
where ik.idtabletopgrips = tabletopgrips.idtabletopgrips and tabletopgrips.idtabletopplacements = tabletopplacements.idtabletopplacements
and tabletopplacements.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement and freetabletopplacement.idobject = 1 and ik.idrobot = 2