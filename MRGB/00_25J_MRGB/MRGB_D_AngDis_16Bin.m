%% Initialization
clear;
close all;

load('C_25J_MRGB_ProcessedData.mat');

displacement_interval = 10;

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

jRightArm = 4:5;
jLeftArm = 7:8;
jRightLeg = 11:12;
jLeftLeg = 14:15;
jFullBody = [1:8 10:21];

H_allJoints16 = {};
H_jRightElbow16 = {};
H_jRightWrist16 = {};
H_jLeftElbow16 = {};
H_jLeftWrist16 = {};
H_jRightKnee16 = {};
H_jRightAnkle16 = {};
H_jLeftKnee16 = {};
H_jLeftAnkle16 = {};

for i=1:numBin
    rM = getRotationMat(i*theta);
    newVec = rM*vec;
end

binVec = zeros(2,numBin);
theta = (2*pi)/numBin;
vec = [0,1]'; 
for i=1:numBin
    rM = getRotationMat(i*theta);
    binVec(:,i) = rM*vec;
end


%% Angular Displacement

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmRange = 1+displacement_interval:displacement_interval:length(allFinalPose{vid}(1,:))-1;
        frmLength = length(frmRange);        
        
        curHist = zeros(1,numBin);
        
        for frm = frmRange
            for jID = [allJoints]
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(pID(jID),1:2);
                oldbone = allFinalPose{vid}{frm-displacement_interval}(jID,1:2) - allFinalPose{vid}{frm-displacement_interval}(pID(jID),1:2);
                
                if norm(bone)>0 && norm(oldbone)>0
                    bone = bone/norm(bone);
                    oldbone = oldbone/norm(oldbone);
                    u = bone;
                    v = oldbone;
                    
                    
                    CosTheta = dot(u,v)/(norm(u)*norm(v));
                    ThetaInDegrees = acosd(CosTheta);
                    
                    for aa=1:numBin
                        if (ThetaInDegrees < AngDisBins(aa))
                            bestBin = aa;
                            break;
                        end
                    end
                    
                    curHist(bestBin) = curHist(bestBin) + 1;
                end
            end
        end
        
        normHist = curHist/sum(curHist);
        H_allJoints16{vid}{allJoints} = normHist;
    end
    
H_jRightElbow16 {vid} = [H_allJoints16{vid}{min(jRightArm)}];
H_jRightWrist16 {vid} = [H_allJoints16{vid}{max(jRightArm)}];

H_jLeftElbow16 {vid} = [H_allJoints16{vid}{min(jLeftArm)}];
H_jLeftWrist16 {vid} = [H_allJoints16{vid}{max(jLeftArm)}];

H_jRightKnee16 {vid} = [H_allJoints16{vid}{min(jRightLeg)}];
H_jRightAnkle16 {vid} = [H_allJoints16{vid}{max(jRightLeg)}];

H_jLeftKnee16 {vid} = [H_allJoints16{vid}{min(jLeftLeg)}];
H_jLeftAnkle16 {vid} = [H_allJoints16{vid}{max(jLeftLeg)}];   
    
end

%% Reshaping for remaining pipeline

AngDisp_RightElbow16 = {};
AngDisp_RightElbow16  = reshape(cell2mat(H_jRightElbow16), (numBin), (numel(vidRange)))';

AngDisp_RightWrist16 = {};
AngDisp_RightWrist16  = reshape(cell2mat(H_jRightWrist16), (numBin), (numel(vidRange)))';

AngDisp_RightKnee16 = {};
AngDisp_RightKnee16  = reshape(cell2mat(H_jRightKnee16), (numBin), (numel(vidRange)))';

AngDisp_RightAnkle16 = {};
AngDisp_RightAnkle16  = reshape(cell2mat(H_jRightAnkle16), (numBin), (numel(vidRange)))';

AngDisp_LeftElbow16 = {};
AngDisp_LeftElbow16  = reshape(cell2mat(H_jLeftElbow16), (numBin), (numel(vidRange)))';

AngDisp_LeftWrist16 = {};
AngDisp_LeftWrist16  = reshape(cell2mat(H_jLeftWrist16), (numBin), (numel(vidRange)))';

AngDisp_LeftKnee16 = {};
AngDisp_LeftKnee16  = reshape(cell2mat(H_jLeftKnee16), (numBin), (numel(vidRange)))';

AngDisp_LeftAnkle16 = {};
AngDisp_LeftAnkle16  = reshape(cell2mat(H_jLeftAnkle16), (numBin), (numel(vidRange)))';

% AngDisp_FullBody16 = {};
% AngDisp_FullBody16  = reshape(cell2mat(H_jFullBody16), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_MRGB_AngDisp_' num2str(numBin) 'bins' '.mat'], 'AngDisp_RightElbow16', 'AngDisp_RightWrist16', 'AngDisp_RightKnee16', 'AngDisp_RightAnkle16', 'AngDisp_LeftElbow16', 'AngDisp_LeftWrist16', 'AngDisp_LeftKnee16', 'AngDisp_LeftAnkle16');

%% Functions
function [rM] = getRotationMat(angle)
    rM = [ cos(angle)   -sin(angle) ; sin(angle)  cos(angle) ];
end