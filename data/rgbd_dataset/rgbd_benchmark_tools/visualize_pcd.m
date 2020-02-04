matlab_pcd = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_ds/1305031453.359684.pcd');
dso_pcd = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_dso/1305031453.359684.pcd');

figure(1)
pcshow(dso_pcd, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')

figure(2)
pcshow(matlab_pcd, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')