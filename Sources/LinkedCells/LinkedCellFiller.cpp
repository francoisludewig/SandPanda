#include "../../Includes/LinkedCells/LinkedCellFiller.h"

#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Solids/Sphere.h"

void LinkedCellFiller::Fill(std::vector<Sphere> & sph, const Configuration& dat, std::vector<Sphere*>& cell) noexcept {
	int x,y,z;
	// Initialisation du tableau tdl
	for(auto& sphere : sph)
		sphere.TDL(nullptr);

	// Initialisation du tableau Cell
	for(int i = dat.Ncellmax ; i--;)
		cell[i] = nullptr;

	// Classement des grains dans les cellules

	// TODO use classic loop for
	// Actually, if change order of the loop, the results change due to double precision
	for(int i = sph.size() ; i--;){
		auto& sphere = sph[i];

		if(sphere.HollowballNum() == -9){
			x = (int)((sphere.X() - dat.xmin)/dat.ax);
			y = (int)((sphere.Y() - dat.ymin)/dat.ay);
			z = (int)((sphere.Z() - dat.zmin)/dat.az);
			if(x < dat.Nx && y < dat.Ny && z < dat.Nz && x >= 0 && y >= 0 && z >= 0){
				//printf("Found it\n");
				//printf("%d in %d\n",i,x*dat.Ny*dat.Nz+y*dat.Nz+z);
				sphere.TDL(cell[x*dat.Ny*dat.Nz+y*dat.Nz+z]);
				cell[x*dat.Ny*dat.Nz+y*dat.Nz+z] = &sphere;
			}
		}
	}
}
