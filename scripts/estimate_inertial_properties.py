#!/usr/bin/env python3

import sys
from os import path, listdir

import trimesh
from pcg_gazebo.parsers.sdf import SDF, create_sdf_element
# Note: Both `trimesh` and `pcg_gazebo` can be installed via `pip`
# `trimesh` is used to estimate volume and inertial properties from meshes of links
# `pcg_gazebo` is used for its SDF parser

# Warning: This script might fail with the current material

def main():

    # Total mass taken from datasheet (given as ~4.2kg)
    # You can also use your own estimate of total mass if you managed to weigh Shadow Hand yourself :)
    total_mass = 4.2
    if len(sys.argv) > 1:
        if float(sys.argv[1]) > 0.0:
            total_mass = float(sys.argv[1])
        else:
            print("Error: Total mass of Shadow Hand (first argument) must be positive.")
            exit(1)
    print('Estimating inertial properties for each link to add up to %f kg' % total_mass)
    
    # Proportion of forearm mass to evenly redistribute to other links
    forearm_mass_redistribute_percentage = 0.5

    # Get path to all visual meshes
    visual_mesh_dir = path.join(path.dirname(path.dirname(
        path.realpath(__file__))), 'shadow_hand', 'meshes', 'visual')
    visual_mesh_basenames = listdir(visual_mesh_dir)
    visual_mesh_basenames.sort()

    # Load all meshes
    meshes = {}
    for mesh_basename in visual_mesh_basenames:
        link_name = path.splitext(mesh_basename)[0]
        mesh_path = path.join(visual_mesh_dir, mesh_basename)
        meshes[link_name] = trimesh.load(mesh_path,
                                         force='mesh',
                                         ignore_materials=True)

    # Compute the total volume of the hand in order to estimate the required density
    total_volume = 0.0
    for link_name in meshes:
        mesh = meshes[link_name]
        print('Volume estimate of %s: %f m^3' % (link_name, mesh.volume))
        if 'finger' in link_name or 'knuckle' in link_name:
            total_volume += 4*mesh.volume
            print(f'Note: {link_name} volume added four times to the total volume')
        else:
            total_volume += mesh.volume
        if 'forearm' in link_name:
            forearm_volume = mesh.volume

    # Compute average density
    average_density = total_mass/total_volume
    print('Average density estimate: %f kg/m^3' % average_density)

    # Determine how much of this is from the forearm and redistribute to other links
    forearm_volume_proportion = forearm_volume/total_volume
    other_links_proportion = 1-forearm_volume_proportion

    # Estimate inertial properties for each link
    mass = {}
    inertia = {}
    centre_of_mass = {}
    for link_name in meshes:
        mesh = meshes[link_name]
        if 'forearm' in link_name:
            mesh.density = (1-forearm_mass_redistribute_percentage)*average_density
        else:
            mesh.density = average_density+((forearm_mass_redistribute_percentage/other_links_proportion-forearm_mass_redistribute_percentage)*average_density)
        mass[link_name] = mesh.mass
        inertia[link_name] = mesh.moment_inertia
        centre_of_mass[link_name] = mesh.center_mass

    # Create a new SDF with one model
    sdf = SDF()
    sdf.add_model(name='shadow_hand')
    model = sdf.models[0]

    total_mass = 0
    for link_name in meshes:
        if 'finger' in link_name or 'knuckle' in link_name:
            total_mass += 4*mass[link_name]
        else:
            total_mass += mass[link_name]
    print(total_mass)

    # Set inertial properties for each link into the SDF
    for link_name in meshes:
        link = create_sdf_element('link')
        link.mass = mass[link_name]
        link.inertia.ixx = inertia[link_name][0][0]
        link.inertia.iyy = inertia[link_name][1][1]
        link.inertia.izz = inertia[link_name][2][2]
        link.inertia.ixy = inertia[link_name][0][1]
        link.inertia.ixz = inertia[link_name][0][2]
        link.inertia.iyz = inertia[link_name][1][2]
        link.inertial.pose = [centre_of_mass[link_name][0],
                              centre_of_mass[link_name][1],
                              centre_of_mass[link_name][2],
                              0.0, 0.0, 0.0]
        model.add_link(link_name, link)

    # Write into output file
    output_file = 'shadow_hand_inertial_out.sdf'
    sdf.export_xml(output_file)
    print('Results written into "%s"' % output_file)


if __name__ == "__main__":
    main()
