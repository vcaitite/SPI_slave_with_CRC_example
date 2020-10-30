# SPI_slave_with_CRC_example

Introduction: This program was developed as a work in the discipline of Embedded Systems Programming at UFMG - Prof. Ricardo de Oliveira Duarte - Department of Electronic Engineering. The program was developed and tested with Stm32F401RE (Nucleo 64 board).

O intuito desse programa é mostrar de maneira didática a utilidade e importância da unidade de cálculo do CRC. O programa foi desenvolvido para a placa Stm32F401RE (Nucleo 64) e se trata de um SPI slave que ao receber uma mensagem do SPI master verifica o CRC e caso esteja correto imprime a mensagem (que pode ser vista de um terminal serial). O código referente ao SPI master se encontra em: https://github.com/vcaitite/SPImasterwithCRC.

Os passos executados por esse programa são:

[1.] Criação de um buffer para recebimento de dados.

[2.] Inicialização da SPI no modo slave.

[3.] Inicialização do periférico de cálculo do CRC.

[4.] Recepção do conjunto de dados estipulado.

[5.] Calculo o CRC do conjunto de dados que corresponde a mensagem.

[6.] Comparação do valor calculado com o recebido (no final dos dados enviados).

[7.] Descarte da mensagem caso a comparação não corresponder. Ou, caso contrário, \textit{print} da mensagem, podendo essa ser visualizada em um terminal serial. 

[8.] Volta para o item 4.
