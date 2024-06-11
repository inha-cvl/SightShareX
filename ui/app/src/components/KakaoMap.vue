
<template>
  <div class="kakao-map" ref="mapContainer"></div>
</template>

<script>
export default {
  name: 'KakaoMap',
  mounted() {
    // Kakao Map API 스크립트 로드
    const script = document.createElement('script');
    script.onload = () => this.initMap();
    script.src = `https://dapi.kakao.com/v2/maps/sdk.js?appkey=e3940dcf53214bac69a3f0109a886ae9&autoload=false`;
    document.head.appendChild(script);
  },
  methods: {
    initMap() {
      // Kakao Maps API가 로드된 후에 'kakao' 객체를 사용할 수 있습니다.
      window.kakao.maps.load(() => {
        const container = this.$refs.mapContainer;
        const options = {
          center: new window.kakao.maps.LatLng(37.384402, 126.655148),
          level: 1
        };
        // 지도 생성
        new window.kakao.maps.Map(container, options);
      });
    }
  }
};
</script>

<style scoped>
.kakao-map {
  width: 100%;
  height: 100%; /* 부모의 높이를 전체로 사용 */
}
</style>


<!-- app key: e3940dcf53214bac69a3f0109a886ae9 -->