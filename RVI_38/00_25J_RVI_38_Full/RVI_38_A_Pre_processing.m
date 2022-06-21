%% Pre-processing the data
clear;

vidRange = 1:38;
jointRange = [1:15, 20:25];


for vid=vidRange
    
    fileRef = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_*_keypoints.json', vid);
    fileA = dir(fileRef);
    frmLength =length(fileA)-1;
    
    idX = [0:frmLength];
       
    frmRange = 0:frmLength;
    
    allPose{vid} = {};
    
    jTrajectory{vid} = zeros(25,3,length (frmRange));
         
      for frm=frmRange
        fname = sprintf('25J_RVI38_Full_Processed/RVI_38_%04i_%012i_keypoints.json', vid, frm);
        val = jsondecode(fileread(fname));


        pose = reshape(val.people(1).pose_keypoints_2d, 3, 25)';
        
        allPose{vid}{frm+1} = pose;
                
        jTrajectory{vid}(:,:,frm+1) = pose;
                   
        iPose {vid} {frm+1} = {};
    end
    
     
    for jID = jointRange
    
        allConfi = reshape(jTrajectory{vid}(jID,3,:), 1,length (frmRange));
        allX = reshape(jTrajectory{vid}(jID,1,:), 1,length (frmRange));
        allY = reshape(jTrajectory{vid}(jID,2,:), 1,length (frmRange));
        M = mean(allConfi);


        exIdx = find(allConfi>=(M-0.07));


        %Interpolation X Value Calculation
        inPt = allX(exIdx);
        inIdx = idX(exIdx);
        xx = [1:length(idX)];        

        INTxx = interp1(inIdx,inPt,xx, 'makima');        
        SmoothXX = movmean(INTxx,5);
        

        %Interpolation Y Value Calculation
        inPt = allY(exIdx);
        inIdx = idX(exIdx);
        yy = [1:length(idX)];
        
        INTyy = interp1(inIdx,inPt,yy, 'makima');       
        SmoothYY = movmean(INTyy,5);

        correctedJoint = zeros(25,3,length (frmRange));
        correctedJoint(jID,1,:) = SmoothXX;
        correctedJoint(jID,2,:) = SmoothYY;
        correctedJoint(jID,3,:) = allConfi;          

            for frm=frmRange 
            iPose {vid} {frm+1} {jID} = (correctedJoint((jID),:,(frm+1)));
            end
    
    end    
    
    
end

%% Exporting the Corrected Pose Data

 save('A_25J_RVI_38_interpPoseData.mat', 'iPose');