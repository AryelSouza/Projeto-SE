# Painel de Visualização 3D - Grafana

Este diretório contém os códigos-fonte (HTML, CSS e JavaScript) responsáveis pela renderização da visualização 3D customizada utilizada no Grafana.

Esta visualização utiliza o plugin **Business Text** para execução de scripts avançados.

## Estrutura dos Arquivos

| Arquivo | Descrição |
| :--- | :--- |
| **`index.html`** | Estrutura base do painel. Contém o elemento `canvas` ou `div` onde a cena 3D é renderizada. |
| **`script.js`** | Lógica da aplicação. Responsável pela inicialização da cena, imports (Three.js, etc.) e manipulação dos dados. |
| **`style.css`** | Estilização do container e elementos de UI do painel. |

## ⚙️ Pré-requisitos

Para que este painel funcione, é necessário instalar o seguinte plugin no seu Grafana:

* **Plugin:** Business Text (ou *Volkov Labs - Text Panel*)
* **Instalação via CLI:**
    ```bash
    grafana-cli plugins install volkovlabs-text-panel
    ```
    *(Reinicie o servidor do Grafana após a instalação)*

## Como Implementar no Dashboard

1. **Adicionar Painel:**
   - Crie um novo painel e, na lista de visualizações, selecione **Business Text** (ou *Text* da Volkov Labs).

2. **Configurar o Código:**
   O plugin geralmente oferece abas ou campos separados para cada tipo de código, o que organiza melhor o projeto:

   - **HTML / Content:** Copie o conteúdo de `index.html` e cole na área principal de conteúdo.
   - **JavaScript / Code:** Copie o conteúdo de `script.js` e cole na seção de código (ou *After Content Render* / *Code execution*).
   - **CSS / Styles:** Copie o conteúdo de `style.css` e cole na seção de estilos CSS.

   > **Dica:** Se o plugin permitir "External Resources" (Recursos Externos), adicione os links das bibliotecas (como Three.js) diretamente nas configurações do painel, em vez de usar tags `<script src="...">` no HTML.

3. **Acesso aos Dados:**
   - O JavaScript neste plugin geralmente acessa os dados através de objetos expostos pelo plugin (ex: `data`, `panelData` ou `context`). Verifique se o `script.js` está consumindo as variáveis corretamente conforme a documentação do plugin Business Text.

## Dependências

Este projeto depende das seguintes bibliotecas (carregadas via CDN ou locais):
* [Three.js](https://threejs.org/) (Exemplo)
* [JQuery](https://jquery.com/) (Se aplicável)

---
*Última atualização: [15/12/2025]*
