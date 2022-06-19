function imgOut = plot3Daxis2image(T, len, K, img, dist)
% Plots a projected RGB 3D axis to a 2D image taken by a camera
% by applying an extrinsic transformation.
% INPUTS:
%       T - extrinsic transformation from world coordinates to camera
%       coordinates
%       len - length of the axis in world coordinates

%create homogenous points representing the edges of the basis vectors for
%the 3D axis AND scale according to len
axis3Dhom = [[0;0;0],len .* eye(3)];
axis3Dhom = [axis3Dhom; 1, 1, 1, 1];

%project points to image coordinates
axis3DPix = projectpoints(axis3Dhom', K, T, dist, size(img, [1,2]));

%Edges of basis vectors in image coordinates
oPix = axis3DPix(1,1:2);
xPix = axis3DPix(2,1:2);
yPix = axis3DPix(3,1:2);
zPix = axis3DPix(4,1:2);

%plot the lines of the axis in RGB colour format
imgOut = insertShape(img, 'Line', [oPix, xPix], 'LineWidth', 5, 'Color','red');
imgOut = insertShape(imgOut, 'Line', [oPix, yPix], 'LineWidth', 5, 'Color','green');
imgOut = insertShape(imgOut, 'Line', [oPix, zPix], 'LineWidth', 5, 'Color','blue');

end

