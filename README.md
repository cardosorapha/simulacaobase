# Código base para simulação no FIRA-SIM

Estes códigos são modificações e adições ao simulador adaptado pelo grupo [RobôCIN](https://robocin.com.br/). Este programa é feito pensando em utilizar funções simples de movimentação do robô permitindo rápida criação de estratégias num ambiente separado ao resto do código que pode parecer complexo demais para quem não o entende por completo. 

## Pré-Requisitos

* Computador com Linux
* [QT](https://www.qt.io/download)
* GIT
```
sudo apt install git
```
Se não for esse comando, você usa outra distribuição Linux e sabe o que fazer
* [FIRASim](https://github.com/robocin/FIRASim) disponível no repositório do RobôCIN
* [FIRA-Client](https://github.com/robocin/fira-client) disponível no repositório do RobôCIN

## Uso

Após instalar corretamente os pré-requisitos, basta clonar este repositório na pasta em que está o arquivo main.cpp do FIRA-Client.

```
cd diretorio/de/instalacao
git init
git remote add origin https://github.com/cardosorapha/simulacaobase
git fetch
git checkout -t origin/master -f
```
Após clonar os arquivos corretamente, eles já serão adicionados à sua versão do simulador. Neste momento você deve criar um branch com seu nome

```
git branch raphael
git checkout raphael
```

## Compartilhando código

Após sessões de programação, você pode atualizar os arquivos **no seu branch**, com o terminal na pasta onde está o main 

```
git add main.cpp strategy.cpp strategy.h
git commit -m "Comentários"
git push
```

Isso fará com que qualquer pessoa possa facilmente testar seu código no computador deles, além de que todo o progresso fica guardado. Note que apenas esses três arquivos serão modificados e atualizados, pois basta modificarmos eles para fazer qualquer programa funcional. Caso alguma modificação em outro arquivo seja necessária, entre em contato comigo e poderemos resolver isso.

## Atualizando o código oficial

A fazer!

## Contato

Raphael Cardoso - cardosodeoliveir@gmail.com

Link do Projeto: [https://github.com/cardosorapha/simulacaobase](https://github.com/cardosorapha/simulacaobase)


