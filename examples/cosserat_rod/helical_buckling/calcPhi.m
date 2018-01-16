function phi = calcPhi(tangent)

theta = acos(tangent(:,3));
thetaMax = max(theta);
phi = (cos(theta)-cos(thetaMax))/(1-cos(thetaMax));

end

