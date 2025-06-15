import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useGLTF } from '@react-three/drei';
export function Eve(props) {
    const { nodes, materials } = useGLTF('/642ec6ea27074836a6a806b03a15ec5a.gltf');
    return (_jsxs("group", { ...props, dispose: null, children: [_jsx("mesh", { geometry: nodes._gltfNode_0.geometry, material: materials.Eve }), _jsx("mesh", { geometry: nodes._gltfNode_2.geometry, material: materials.Eve }), _jsx("mesh", { geometry: nodes._gltfNode_4.geometry, material: materials.Eve }), _jsx("mesh", { geometry: nodes._gltfNode_6.geometry, material: materials.Eve }), _jsx("mesh", { geometry: nodes._gltfNode_8.geometry, material: materials['Mat.2'] }), _jsx("mesh", { geometry: nodes._gltfNode_10.geometry, material: materials.Mat }), _jsx("mesh", { geometry: nodes._gltfNode_12.geometry, material: materials.Mat })] }));
}
useGLTF.preload('/642ec6ea27074836a6a806b03a15ec5a.gltf');
