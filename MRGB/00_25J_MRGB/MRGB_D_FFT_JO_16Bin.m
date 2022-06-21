%% Initialization

clear;
close all;
load('C_25J_MRGB_ProcessedData.mat');
vidRange = 1:12;
numBin = 16;
pwr = 20;

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


%% FFTJO

for vid = vidRange
    
    for allJoints = jFullBody
        
        frmLength = length(allFinalPose{vid}(1,:))-1;
        frmRange = 1:frmLength;
        
        FFTJO_curHist = zeros(1,numBin);
        
        for frm = frmRange
            for jID = [allJoints]
                bone = allFinalPose{vid}{frm}(jID,1:2) - allFinalPose{vid}{frm}(pID(jID),1:2); 
                
                a = bone(1);
                b = bone(2);
                c = a^2 + b^2;
                if (c==0)
                    CosTheta = 0;
                else
                    CosTheta = a/sqrt(a^2 + b^2);
                end
                ThetaInDegrees = acosd(CosTheta);
                Orientation (frm) = ThetaInDegrees;
                
            end
        end
     
        Fs = 50;
        T = 1/Fs;
        L = length(Orientation);
        t=(0:L-1)*T;
        
        Y = fft(Orientation);
        P2 = abs(Y/L);
        P1 = P2 (1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
        
        f = Fs*(0:(L/2))/L;        
        maxF = max(f);
        
        for aa = 1:length(f)
 
            
           if f(aa) < maxF*(1^pwr/numBin^pwr)
                bestBin = 1;
            elseif f(aa) < maxF*(2^pwr/numBin^pwr)
                bestBin = 2;
            elseif f(aa) < maxF*(3^pwr/numBin^pwr)
                bestBin = 3;
            elseif f(aa) < maxF*(4^pwr/numBin^pwr)
                bestBin = 4;
            elseif f(aa) < maxF*(5^pwr/numBin^pwr)
                bestBin = 5;
            elseif f(aa) < maxF*(6^pwr/numBin^pwr)
                bestBin = 6;
            elseif f(aa) < maxF*(7^pwr/numBin^pwr)
                bestBin = 7;
            elseif f(aa) < maxF*(8^pwr/numBin^pwr)
                bestBin = 8;
            elseif f(aa) < maxF*(9^pwr/numBin^pwr)
                bestBin = 9;
            elseif f(aa) < maxF*(10^pwr/numBin^pwr)
                bestBin = 10;
            elseif f(aa) < maxF*(11^pwr/numBin^pwr)
                bestBin = 11;
            elseif f(aa) < maxF*(12^pwr/numBin^pwr)
                bestBin = 12;
            elseif f(aa) < maxF*(13^pwr/numBin^pwr)
                bestBin = 13;
            elseif f(aa) < maxF*(14^pwr/numBin^pwr)
                bestBin = 14;
            elseif f(aa) < maxF*(15^pwr/numBin^pwr)
                bestBin = 15;
            elseif f(aa) <= maxF
                bestBin = 16;
           end

            FFTJO_curHist(bestBin) = FFTJO_curHist(bestBin) + P1(aa);
            
        end
        
        FFTJO_normHist = FFTJO_curHist/sum(FFTJO_curHist);        
        
        H_allJoints16{vid}{allJoints} = FFTJO_normHist;
        
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

FFTJO_RightElbow16 = {};
FFTJO_RightElbow16  = reshape(cell2mat(H_jRightElbow16), (numBin), (numel(vidRange)))';

FFTJO_RightWrist16 = {};
FFTJO_RightWrist16  = reshape(cell2mat(H_jRightWrist16), (numBin), (numel(vidRange)))';

FFTJO_RightKnee16 = {};
FFTJO_RightKnee16  = reshape(cell2mat(H_jRightKnee16), (numBin), (numel(vidRange)))';

FFTJO_RightAnkle16 = {};
FFTJO_RightAnkle16  = reshape(cell2mat(H_jRightAnkle16), (numBin), (numel(vidRange)))';

FFTJO_LeftElbow16 = {};
FFTJO_LeftElbow16  = reshape(cell2mat(H_jLeftElbow16), (numBin), (numel(vidRange)))';

FFTJO_LeftWrist16 = {};
FFTJO_LeftWrist16  = reshape(cell2mat(H_jLeftWrist16), (numBin), (numel(vidRange)))';

FFTJO_LeftKnee16 = {};
FFTJO_LeftKnee16  = reshape(cell2mat(H_jLeftKnee16), (numBin), (numel(vidRange)))';

FFTJO_LeftAnkle16 = {};
FFTJO_LeftAnkle16  = reshape(cell2mat(H_jLeftAnkle16), (numBin), (numel(vidRange)))';

% FFTJO_FullBody16 = {};
% FFTJO_FullBody16  = reshape(cell2mat(H_jFullBody16), (numBin), (numel(vidRange)))';


%% Export all Histogram data (HOJD2D) for remaining pipeline
save(['25J_MRGB_FFT_JO_' num2str(numBin) 'bins' '.mat'], 'FFTJO_RightElbow16', 'FFTJO_RightWrist16', 'FFTJO_RightKnee16', 'FFTJO_RightAnkle16', 'FFTJO_LeftElbow16', 'FFTJO_LeftWrist16', 'FFTJO_LeftKnee16', 'FFTJO_LeftAnkle16');