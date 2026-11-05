// three.js scene built from the MuJoCo model. MuJoCo is z-up, three.js y-up: (x, y, z) -> (x, z, -y).
// Draws geom groups 0-2 (export_web.py puts screws, pins etc. in group 3). Types used here: plane, sphere, box, cylinder, mesh.
import * as THREE from 'three';
import { toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const pos = (b, i, t) => t.set(b[3 * i], b[3 * i + 2], -b[3 * i + 1]);
export const quat = (b, i, t) => t.set(-b[4 * i + 1], -b[4 * i + 3], b[4 * i + 2], -b[4 * i]);
export const toMj = v => [v.x, -v.z, v.y];
export const fromMj = (x, y, z) => new THREE.Vector3(x, z, -y);

export function buildScene(mj, model, scene) {
  const T = mj.mjtGeom, bodies = {}, meshCache = {}, matCache = {};
  for (let g = 0; g < model.ngeom; g++) {
    if (model.geom_group[g] > 2) continue;
    const b = model.geom_bodyid[g], type = model.geom_type[g];
    const s = [model.geom_size[3 * g], model.geom_size[3 * g + 1], model.geom_size[3 * g + 2]];
    const rgba = [0, 1, 2, 3].map(k => model.geom_rgba[4 * g + k]);
    if (!bodies[b]) { bodies[b] = new THREE.Group(); bodies[b].bodyID = b; scene.add(bodies[b]); }
    let geo, mat;
    if (type === T.mjGEOM_PLANE.value) {
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6).rotateX(-Math.PI / 2),
        new THREE.MeshStandardMaterial({ color: 0xd9dee3, roughness: 0.95 }));
      floor.receiveShadow = true; pos(model.geom_pos, g, floor.position); bodies[b].add(floor);          // the plane's height (the Connect 4 floor is below the table)
      const grid = new THREE.GridHelper(6, 60, 0xb4bcc4, 0xc6cdd4); grid.position.y = floor.position.y + 0.0005; bodies[b].add(grid);
      continue;
    }
    if (type === T.mjGEOM_SPHERE.value) geo = new THREE.SphereGeometry(s[0], 40, 24);
    else if (type === T.mjGEOM_BOX.value) geo = new THREE.BoxGeometry(2 * s[0], 2 * s[2], 2 * s[1]);
    else if (type === T.mjGEOM_CYLINDER.value) geo = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 48);      // MuJoCo's z axis becomes three's y
    else if (type === T.mjGEOM_MESH.value) {
      const id = model.geom_dataid[g];
      if (!meshCache[id]) {
        const v = model.mesh_vert.slice(3 * model.mesh_vertadr[id], 3 * (model.mesh_vertadr[id] + model.mesh_vertnum[id]));
        for (let i = 0; i < v.length; i += 3) { const y = v[i + 1]; v[i + 1] = v[i + 2]; v[i + 2] = -y; }
        const f = model.mesh_face.slice(3 * model.mesh_faceadr[id], 3 * (model.mesh_faceadr[id] + model.mesh_facenum[id]));
        let gm = new THREE.BufferGeometry();
        gm.setAttribute('position', new THREE.BufferAttribute(new Float32Array(v), 3)); gm.setIndex(Array.from(f));
        meshCache[id] = toCreasedNormals(gm, Math.PI / 5);      // sharp CAD edges, smooth curved faces
      }
      geo = meshCache[id];
    } else continue;
    const transparent = rgba[3] < 0.99;
    const mkey = rgba.join(',');                                    // one material per colour: far fewer state changes per frame
    mat = matCache[mkey] || (matCache[mkey] = new THREE.MeshStandardMaterial({ color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: 0.6, metalness: 0.05,
      transparent, opacity: rgba[3], side: THREE.DoubleSide }));
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = !transparent; mesh.receiveShadow = true; mesh.bodyID = b;
    pos(model.geom_pos, g, mesh.position); quat(model.geom_quat, g, mesh.quaternion);
    bodies[b].add(mesh);
  }
  return bodies;
}

export function syncBodies(data, bodies) {
  for (const b in bodies) { const i = +b; pos(data.xpos, i, bodies[b].position); quat(data.xquat, i, bodies[b].quaternion); }
}
