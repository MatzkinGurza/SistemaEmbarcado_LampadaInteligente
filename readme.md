
# Sistema Embarcado - Lâmpada Inteligente

Este repositório contém o código-fonte e a documentação do projeto de uma Lâmpada Inteligente, desenvolvido como parte da disciplina **PMR3402 - Sistemas Embarcados** da Escola Politécnica da Universidade de São Paulo (Poli-USP).

## 📜 Visão Geral do Projeto

O objetivo deste projeto é oferecer uma solução de iluminação inteligente e automatizada, capaz de ajustar dinamicamente a intensidade de uma lâmpada artificial para complementar a luz natural de um ambiente. O sistema visa não apenas melhorar o conforto visual dos usuários , mas também promover a eficiência energética ao evitar o uso desnecessário de eletricidade.

A arquitetura é modular e dividida em duas unidades principais que se comunicam sem fio utilizando o protocolo **ESP-NOW** :

1.  **Módulo de Sensoriamento**: Equipado com um sensor de luminosidade BH1750, atua como o "olho" do sistema, medindo continuamente a luz do ambiente.
2.  **Módulo de Iluminação**: O "cérebro" e "músculo" do sistema. Recebe os dados de luminosidade, processa a lógica de controle e ajusta o brilho de uma lâmpada dimerizável através de um módulo dimmer com TRIAC.

## ✨ Arquitetura de Software: Foco na Máquina de Estados

O núcleo lógico do **Módulo de Iluminação** é governado por uma **máquina de estados** robusta. Essa máquina de estados gerencia os diferentes modos de operação do sistema, como o controle `Manual` (via potenciômetro) e `Automático` (baseado nos dados do sensor).

### A Dupla Abordagem de Implementação

A lógica central, baseada em uma máquina de estados para controlar os modos de operação, é suficientemente simples para ser implementada sem a necessidade estrita de um sistema operacional de tempo real (RTOS) como o FreeRTOS.

No entanto, com o objetivo de explorar e comparar diferentes paradigmas de desenvolvimento para sistemas embarcados, **o projeto foi intencionalmente implementado de duas formas distintas**:

#### 1\. Modelo Clássico (Interface Arduino `setup()` e `loop()`)

Esta é a abordagem tradicional e mais direta no ecossistema Arduino. A máquina de estados e todas as lógicas de controle (leitura de sensores, atuação no dimmer, comunicação) são gerenciadas dentro do laço principal (`loop()`). Este modelo é eficaz e demonstra o funcionamento completo do sistema de forma sequencial e centralizada.

#### 2\. Modelo com FreeRTOS

Nesta versão, a mesma lógica da máquina de estados foi encapsulada dentro de um ambiente multitarefa utilizando o FreeRTOS. As diferentes responsabilidades do sistema (como gerenciar a comunicação ESP-NOW, ler as entradas do usuário e controlar a lâmpada) foram separadas em tarefas (`tasks`) distintas que rodam de forma concorrente.

Embora o FreeRTOS não fosse um requisito obrigatório para a funcionalidade do projeto, esta implementação serve como um estudo prático sobre como estruturar um sistema embarcado para ser mais escalável, modular e responsivo, preparando o terreno para futuras expansões com funcionalidades mais complexas.

## 📂 Estrutura do Repositório (Sugestão)

```
.
├── /src
│   ├── /lampada_inteligente_classic   # Implementação com setup() e loop()
│   └── /lampada_inteligente_freertos  # Implementação com FreeRTOS
├── /docs
│   └── MANUAL_DO_USUARIO_LAMPADA_INTELIGENTE.pdf # Manual técnico do projeto
└── README.md                          # Este arquivo
```

## 🧑‍💻 Autores

  * Eduardo da Silva Bauer Guimarães
  * Luís Eduardo Dorneles Fauth
  * Mateus Matzkin Gurza
  * Vinícius Barile Lora Franco

-----
