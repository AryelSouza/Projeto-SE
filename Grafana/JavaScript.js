(async function () {

  // --- 1. IMPORTAÇÃO DE BIBLIOTECAS ---
  // Importa o Three.js (motor 3D), o Carregador de arquivos GLTF/GLB e os Controles de Órbita via CDN (esm.sh).
  const THREE = await import('https://esm.sh/three@0.160.0');
  const { GLTFLoader } = await import('https://esm.sh/three@0.160.0/examples/jsm/loaders/GLTFLoader.js');
  const { OrbitControls } = await import('https://esm.sh/three@0.160.0/examples/jsm/controls/OrbitControls.js');

  // Obtém o elemento HTML onde o canvas 3D será desenhado.
  const container = document.getElementById('canvas-container');
  if (!container) return; // Se o container não existir, para a execução.

  // --- 2. CONFIGURAÇÃO DA CENA 3D (RODA APENAS UMA VEZ) ---
  // O Grafana recarrega este script sempre que os dados mudam. 
  // A verificação abaixo garante que não recriemos a cena do zero a cada atualização,
  // mantendo a performance e evitando duplicação de canvas.
  if (!container.sceneState) {
    const width = container.clientWidth || 400;
    const height = container.clientHeight || 300;

    // Cria a cena básica
    const scene = new THREE.Scene();

    // --- ILUMINAÇÃO ---
    // Luz ambiente (ilumina tudo igualmente)
    const ambientLight = new THREE.AmbientLight(0xffffff, 1.5);
    scene.add(ambientLight);

    // Luz direcional (simula o sol, cria sombras e profundidade)
    const dirLight = new THREE.DirectionalLight(0xffffff, 2.0);
    dirLight.position.set(5, 10, 7);
    scene.add(dirLight);

    // --- CÂMERA ---
    // Câmera perspectiva (simula o olho humano)
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
    camera.position.set(0, 25, 30); // Posição inicial (X, Y, Z)
    camera.lookAt(0, 0, 0); // Aponta para o centro

    // --- RENDERIZADOR ---
    // alpha: true permite fundo transparente
    const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement); // Adiciona o canvas ao HTML

    // --- CONTROLES ---
    // Permite girar e dar zoom com o mouse
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Adiciona inércia ao movimento
    controls.enableZoom = true;

    // --- ESTADO GLOBAL ---
    // Salva referências dentro do elemento HTML para serem acessadas nas próximas execuções do script
    container.sceneState = {
      mesh: null,       // Vai guardar o modelo 3D da mesa
      targetPitch: 0,   // O ângulo alvo para onde a mesa deve ir (eixo X)
      targetRoll: 0     // O ângulo alvo para onde a mesa deve ir (eixo Z)
    };

    // --- CARREGAMENTO DO MODELO ---
    const loader = new GLTFLoader();
    loader.load('/public/img/mesa.glb', function (gltf) {
      const mesh = gltf.scene;
      mesh.scale.set(0.1, 0.1, 0.1); // Ajuste de escala se o modelo for muito grande
      scene.add(mesh);
      container.sceneState.mesh = mesh; // Salva o modelo carregado no estado global
    });

    // Função auxiliar para converter Graus em Radianos
    const degToRad = (deg) => deg * (Math.PI / 180);

    // --- LOOP DE ANIMAÇÃO ---
    // Esta função roda ~60 vezes por segundo
    function animate() {
      requestAnimationFrame(animate);
      controls.update();
      const state = container.sceneState;

      // Se o modelo já carregou, atualiza a rotação
      if (state.mesh) {
        const currentX = state.mesh.rotation.x;
        const currentZ = state.mesh.rotation.z;

        // --- INTERPOLAÇÃO (SUAVIZAÇÃO) ---
        // Em vez de pular direto para o novo ângulo, ele move 10% (0.1) da distância a cada frame.
        // Isso cria o efeito "hidráulico" suave visualmente.
        state.mesh.rotation.x = currentX + (degToRad(state.targetPitch) - currentX) * 0.1;
        state.mesh.rotation.z = currentZ + (degToRad(state.targetRoll) - currentZ) * 0.1;
      }
      renderer.render(scene, camera);
    }
    animate();

    // --- REDIMENSIONAMENTO ---
    // Ajusta a câmera se o tamanho da janela do navegador mudar
    const onResize = () => {
      const w = container.clientWidth;
      const h = container.clientHeight;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    window.addEventListener('resize', onResize);
  }

  // === 3. LEITURA DE DADOS E ATUALIZAÇÃO (RODA A CADA UPDATE DO GRAFANA) ===
  try {
    // Busca o elemento escondido onde o Grafana injeta o JSON
    const dataBridge = document.getElementById('grafana-data-source');

    // VERIFICAÇÃO DE SEGURANÇA: Só tenta ler se houver texto válido
    if (dataBridge && dataBridge.textContent && dataBridge.textContent.trim().length > 0) {

      const jsonText = dataBridge.textContent.trim();
      const rawData = JSON.parse(jsonText); // Converte texto em Objeto JS

      let p = 0; // Variável temporária para Pitch
      let r = 0; // Variável temporária para Roll
      let found = false;

      // --- LÓGICA "SHERLOCK HOLMES" ---
      // O Grafana pode enviar dados em formatos diferentes dependendo da versão ou da query.
      // Abaixo tentamos encontrar "pitch" e "roll" de várias formas:

      // CASO 1: Formato padrão de Time Series do Grafana (Objeto 'series')
      if (rawData.series && rawData.series.length > 0) {
        const s = rawData.series[0];
        // Procura nos campos (fields) algo que tenha "pitch" ou "roll" no nome (case insensitive)
        const fPitch = s.fields.find(f => f.name.toLowerCase().includes('pitch'));
        const fRoll = s.fields.find(f => f.name.toLowerCase().includes('roll'));

        if (fPitch && fRoll) {
          // Pega o último valor recebido (o mais atual)
          p = fPitch.values[fPitch.values.length - 1];
          r = fRoll.values[fRoll.values.length - 1];
          found = true;
        }
      }
      // CASO 2: Modo "Table" ou JSON direto
      else if (rawData.data && (rawData.data.pitch !== undefined || rawData.data.roll !== undefined)) {
        p = rawData.data.pitch || 0;
        r = rawData.data.roll || 0;
        found = true;
      }
      // CASO 3: Objeto raiz simples
      else if (rawData.pitch !== undefined || rawData.roll !== undefined) {
        p = rawData.pitch || 0;
        r = rawData.roll || 0;
        found = true;
      }

      // Se encontrou dados válidos, atualiza o alvo (Target) no estado global.
      // O loop 'animate' lá em cima vai perceber essa mudança e mover a mesa suavemente.
      if (found && container.sceneState) {
        container.sceneState.targetPitch = p;
        container.sceneState.targetRoll = r;
      }
    }
  } catch (e) {
    // Se o JSON estiver quebrado ou vazio, ignora o erro para não travar o painel
    if (!e.message.includes("JSON")) {
      console.error("Erro JS na leitura de dados:", e);
    }
  }

})();
