% velocityJacobian - Returns the velocity jacobian of the manipulator given 
%                    an array of links created by the createLink function and 
%                    the current joint variables. 
%
%   Jv = velocityJacobian( linkList, paramList )
%
%       By inputting the link list and the current state parameter list,
%       this function returns the velocity jacobian of this set of links.
%
%   linkList = the array consisting all the link structures, every
%              structure consists all the information need for the link
%   paramList = the array that consists the joint variables

function [H, Jv] = velocityJacobian( linkList, paramList )

A=length(linkList);
H=dhFwdKine(linkList,paramList);

%Putting paramList values in to linkList
z=1;
for n = 1:1:A  
    if linkList(n).isRotary == 1
        linkList(n).theta=paramList(z);
        z=z+1;
    elseif linkList(n).isRotary == 0
        linkList(n).d = paramList(z);
        z=z+1;
    else
    end
end

R0_N = sym(zeros(3,3,A));
TN_0 = sym(zeros(4,4,A));
for i = 1:1:A
    R0_N(:,:,i) = H(1:3,1:3,i);
    TN_0(:,:,i)=simplify(inv(H(:,:,i)));
end

%Prellocating arrays
TN_i=sym(zeros(4,4,A));
z=sym(zeros(3,1,A));
d=sym(zeros(3,1,A));
JvN=sym(zeros(6,A,A));
Jv=sym(zeros(6,A,A));

T_cur=sym(zeros(4,4,A));
for i = 1:1:A
    
    %Getting the list of TN_i
    T_cur(:,:,i) = TN_0(:,:,i);
    for n=1:1:i
        T_cur(:,:,i)=T_cur(:,:,i)*dhTransform(linkList(n).a,linkList(n).d,linkList(n).alpha,linkList(n).theta);
        TN_i(:,:,n)=T_cur(:,:,i);
    end

    %Getting the list of z from TN_i
    for m=1:1:i
        z(:,:,m)=TN_i(1:3,1:3,m)*[0;0;1];
    end

    %Getting the list of d from TN_i
    for p=1:1:i
        d(:,:,p)=-TN_i(1:3,4,p);
    end

    %Construct JvN with z and d
    for j=1:1:i
        for q=1:1:j
            JvN(:,q,j)=simplify([cross(z(:,:,q),d(:,:,q));z(:,:,q)]);
        end
    end

    %Get Jv from JvN
    for j = 1:1:i
        Jv(:,:,i)=[R0_N(:,:,j) zeros(3,3);zeros(3,3) R0_N(:,:,j)]*JvN(:,:,j);
    end
end