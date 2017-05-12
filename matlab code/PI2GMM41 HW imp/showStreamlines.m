function showStreamlines(GMR,Domain, n_samples)

n_Gauss = length(GMR.priors);

ax.XLim = Domain(1:2);
ax.YLim = Domain(3:4);
nx=n_samples;
ny=n_samples;
ax_x=linspace(ax.XLim(1),ax.XLim(2),nx); %computing the mesh points along each axis
ax_y=linspace(ax.YLim(1),ax.YLim(2),ny); %computing the mesh points along each axis
[x_tmp y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:) y_tmp(:)]';

h = zeros(n_Gauss,1);
xd = zeros(2,length(x));

for i=1:length(x)
    for j=1:n_Gauss
     h(j) = GMR.priors(j) * mvnpdf(x(:,i), GMR.muInput(:,j), GMR.sigmaInput(:,:,j));
     xd(:,i) = xd(:,i) + h(j)*(GMR.A(:,:,j)*x(:,i) + GMR.b(:,j));
    end
end


streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),1,'method','cubic')
axis([ax.XLim ax.YLim]);box on