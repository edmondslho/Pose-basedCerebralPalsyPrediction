%% Initialization
clear;
close all;

load('C_25J_MRGB_ProcessedData.mat');
displacement_interval = 4;
vidRange = 1:12;
numBin = 16;
maxDegree = 180;
AngDisBins = zeros(1,numBin);
AngDisBins(numBin) = maxDegree;
for i = numBin-1:-1:1
    AngDisBins(i) = maxDegree/2;
    maxDegree = AngDisBins(i);
end

theta = (2*pi)/numBin;
vec = [0,1]';
jFullBodyRoot = 1:21;

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jTorso = [1:3 6 9 10 13];
allJoints = [1:15];

H_jFullBody16RelOri = {};


%% Full Body Relative Orientation

for vid = vidRange    
    
    frmLength = length(allFinalPose{vid}(1,:))-1;
    frmRange = 1+displacement_interval:displacement_interval:frmLength;
    
    curHist = {};
    for i=1:length(allJoints)
        curHist{i} = zeros(1,numBin);
    end
    
    for frm = frmRange
       
        for aa = 1:length(allJoints)    
            jID = jFullBodyRoot(allJoints(aa));
            for bb = 1:length(allJoints)
                jID2 = jFullBodyRoot(allJoints(bb));
             
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(jID2,1:2);    % always pointing from parent to child, so in math it will be "child - parent"
                oldbone = allFinalPose{vid}{frm-displacement_interval}(jID,1:2) - allFinalPose{vid}{frm-displacement_interval}(jID2,1:2);    % always pointing from parent to child, so in math it will be "child - parent"

                if norm(bone)>0     
                    bone = bone/norm(bone);
                    oldbone = oldbone/norm(oldbone);
                    
                    u = bone;
                    v = oldbone;
                    
                    
                    CosTheta = dot(u,v)/(norm(u)*norm(v));
                    ThetaInDegrees = acosd(CosTheta);
                    curHist{aa} = curHist{aa} + getHist(ThetaInDegrees, AngDisBins, false);
                end
            end
        end
    end
    
    

    fusedHist = [];
    for i=1:length(allJoints)
        if sum(curHist{i})>0
            curHist{i} = curHist{i}/sum(curHist{i});
        end
        fusedHist = [ fusedHist curHist{i} ];
    end
    
    H_jFullBody16RelOri{vid} = fusedHist;
    
end  


%% Reshaping for remaining pipeline

Rel_JO_AngDis_Parts16 = {};
Rel_JO_AngDis_Parts16  = reshape(cell2mat(H_jFullBody16RelOri), [], (numel(vidRange)))';

HORJAD2D_LW16 = Rel_JO_AngDis_Parts16(:,113:128);
HORJAD2D_LE16 = Rel_JO_AngDis_Parts16(:,97:112);
HORJAD2D_RW16 = Rel_JO_AngDis_Parts16(:,65:80);
HORJAD2D_RE16 = Rel_JO_AngDis_Parts16(:,49:64);
HORJAD2D_LA16 = Rel_JO_AngDis_Parts16(:,225:240);
HORJAD2D_LK16 = Rel_JO_AngDis_Parts16(:,209:224);
HORJAD2D_RA16 = Rel_JO_AngDis_Parts16(:,177:192);
HORJAD2D_RK16 = Rel_JO_AngDis_Parts16(:,161:176);

HORJAD2D_Larm16 = [HORJAD2D_LW16+HORJAD2D_LE16];
HORJAD2D_Lleg16 = [HORJAD2D_LA16+HORJAD2D_LK16];
HORJAD2D_Rarm16 = [HORJAD2D_RW16+HORJAD2D_RE16];
HORJAD2D_Rleg16 = [HORJAD2D_RA16+HORJAD2D_RK16];


%% Export all Histogram data (HOJO2D) for remaining pipeline
save(['25J_MRGB_Rel_JO_AngDis_' num2str(numBin) 'bins' '.mat'], 'Rel_JO_AngDis_Parts16', 'HORJAD2D_Larm16', 'HORJAD2D_Lleg16', 'HORJAD2D_Rarm16','HORJAD2D_Rleg16');

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end