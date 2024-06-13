<template>
  <div id="app">
    <AppHeader />
    <div class="content-container">
      <Vehicle3DView class="vehicle-3d-view" />
      <MainContent class="main-content" />
      <KakaoMap class="map" />
    </div>
  </div>
</template>

<script>
import AppHeader from './components/AppHeader.vue';
import MainContent from './components/MainContent.vue';
import KakaoMap from './components/KakaoMap.vue';
import Vehicle3DView from './components/Vehicle3DView.vue';

export default {
  name: 'App',
  components: {
    AppHeader,
    MainContent,
    KakaoMap,
    Vehicle3DView
  }
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin: 0;
  padding: 0;
  height: 100vh; /* 전체 높이를 사용 */
}
.content-container {
  display: flex;
  height: calc(100vh - 70px); /* 헤더 높이(70px) 제외한 나머지 */
}
.main-content {
  flex: 1; /* 나머지 공간을 차지 */
  text-align: center;
}
.map {
  flex: 1.5;
  height: 100%; /* 페이지 전체 높이를 유지 */
}
.vehicle-3d-view {
  flex: 2;
  height: 100%; /* 페이지 전체 높이를 유지 */
}
</style>
