# Tracery Pro - Aplicativo de Tracking de Vídeo

Tracery Pro é um aplicativo de desktop para tracking de objetos em vídeos, com várias opções de visualização e personalização.

## Requisitos

- Python 3.7 ou superior
- pip (gerenciador de pacotes Python)

## Instalação

1. Clone este repositório ou baixe os arquivos
2. Abra um terminal na pasta do projeto
3. Crie um ambiente virtual (opcional, mas recomendado):
```bash
python -m venv venv
source venv/bin/activate  # No Windows: venv\Scripts\activate
```
4. Instale as dependências:
```bash
pip install -r requirements.txt
```

## Uso

1. Execute o aplicativo:
```bash
python main.py
```

2. No aplicativo:
   - Clique em "Load Video" para carregar um vídeo
   - Selecione o método de tracking desejado
   - Ajuste as configurações conforme necessário
   - Clique em "Play" para iniciar o processamento
   - Use "Save Video" para salvar o resultado

## Funcionalidades

- Múltiplos métodos de tracking (Color Tracking, Motion Detection, Optical Flow, etc.)
- Personalização de cores e estilos
- Visualização de coordenadas
- Diferentes tipos de conexões entre objetos
- Exportação de vídeo processado 