% This function crates a set of demonstrations suitable for initializing
% the policy for PI2, by selecting the relevant dimensions.

function demos = getDemosFromDataset(DataSets)

    for cnt1 = 1:size(DataSets,2)
        demos{cnt1}(1,:) = DataSets{cnt1}.posData(:,1)';
        demos{cnt1}(2,:) = DataSets{cnt1}.posData(:,3)';
        demos{cnt1}(3,:) = DataSets{cnt1}.eulerData(:,1)';
    end
end