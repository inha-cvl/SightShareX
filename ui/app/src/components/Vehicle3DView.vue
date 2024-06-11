<template>
  <div class="vehicle-3d-view" ref="sceneContainer"></div>
</template>

<script>
import * as THREE from 'three';

export default {
  name: 'Vehicle3DView',
  mounted() {
    this.init3DScene();
  },
  methods: {
    init3DScene() {
      // 장면 및 카메라 설정
      const scene = new THREE.Scene();
      const camera = new THREE.PerspectiveCamera(53, this.$refs.sceneContainer.clientWidth / this.$refs.sceneContainer.clientHeight, 0.1, 1000);
      camera.position.set(0,9,14);
      camera.lookAt(new THREE.Vector3(0, 6,10)); 
      scene.background = new THREE.Color(0xffffff); // 흰색 배경

      // 렌더러 설정
      const renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setSize(this.$refs.sceneContainer.clientWidth, this.$refs.sceneContainer.clientHeight);
      this.$refs.sceneContainer.appendChild(renderer.domElement);

      // 바닥 생성
      const planeGeometry = new THREE.PlaneGeometry(20, 50); // 바닥 크기를 조정
      const planeMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide });
      const plane = new THREE.Mesh(planeGeometry, planeMaterial);
      plane.rotation.x = Math.PI / 2;
      scene.add(plane);

      // 차량 생성
      const carGeometry = new THREE.BoxGeometry(2, 1, 4);  // 차량 크기를 크게 조정
      const carMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
      const car = new THREE.Mesh(carGeometry, carMaterial);
      car.position.y = 0.5;  // 차량의 높이 설정
      scene.add(car);

      // 차량 앞의 객체 생성
      const objectGeometry = new THREE.BoxGeometry(2, 1, 4);  // 객체 크기를 크게 조정
      const objectMaterial = new THREE.MeshBasicMaterial({ color: 0xb3b3b3 });
      const object = new THREE.Mesh(objectGeometry, objectMaterial);
      object.position.set(0, 1, -7);  // 객체의 위치 설정
      scene.add(object);

      // 차선 추가
      const lineMaterial = new THREE.MeshBasicMaterial({ color: 0x7a7a7a });
      const lineWidth = 0.1;
      const lineLength = 50;

      const leftLineGeometry = new THREE.BoxGeometry(lineWidth, 0.01, lineLength);
      const rightLineGeometry = new THREE.BoxGeometry(lineWidth, 0.01, lineLength);

      const leftLine = new THREE.Mesh(leftLineGeometry, lineMaterial);
      leftLine.position.set(-2, 0.01, 0);
      scene.add(leftLine);

      const rightLine = new THREE.Mesh(rightLineGeometry, lineMaterial);
      rightLine.position.set(2, 0.01, 0);
      scene.add(rightLine);

      // 애니메이션 설정
      const animate = () => {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
      };
      animate();

      // 윈도우 크기 변경 시 렌더러와 카메라 비율 조정
      window.addEventListener('resize', () => {
        camera.aspect = this.$refs.sceneContainer.clientWidth / this.$refs.sceneContainer.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(this.$refs.sceneContainer.clientWidth, this.$refs.sceneContainer.clientHeight);
      });
    }
  }
};
</script>

<style scoped>
.vehicle-3d-view {
  width: 100%;
  height: 100%;
  background-color: #ffffff; /* 흰색 배경 */
}
</style>
