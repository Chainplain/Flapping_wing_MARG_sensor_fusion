R=rotx(0);
R2=rotx(30);
trplot(R,'frame', 'A');
hold on;

gap_rec = 2000;
omega = [0  pi/2/gap_rec 0];

for i = 1 : gap_rec
    addmatrix =   [ 0,         -omega(3),  omega(2);...
                    omega(3),  0,          -omega(1);...
                    -omega(2), omega(1),   0];
   R = R +  R * addmatrix; 
   error = R(:,1)' * R(:,2);
   R_new_1= R(:,1) - 0.5 * error *  R(:,2);
   R_new_2= R(:,2) - 0.5 * error *  R(:,1);
   R_new_3= cross(R_new_1, R_new_2);
   R = [R_new_1, R_new_2, R_new_3];
   if ( mod(i,200)==0)
     trplot(R,'rgb');
   end
end

