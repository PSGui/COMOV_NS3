# COMOV_NS3

1- git clone https://github.com/PSGui/COMOV_NS3.git COMOV-Project

2- git pull origin main

3- cp COMOV-Project/src/comov.cc ~/COMOV/ns-3/scratch/comov.cc

onde "COMOV-Projec/src/comov.cc" é o que foi clonado e "~/COMOV/ns-3/scratch/comov.cc" deverá de ser uma pasta que contém o ns-3 instalado. O caminho poderá não ser igual.

4- cd ~/COMOV/ns-3

5- ./ns3 run "scratch/comov --nBuses=2 --numPackets=200 --interval=1ms --outputCsv=resultados.csv"

nBuses - Número de autocarros na simulação  
numPackets - Número de pacotes enviados por cada autocarro na simulação  
interval - intervalo de tempo entre o envio de pacotes de cada autocarro  
outputCsv - nome do ficheiro .csv que contém os dados da simulação (pode ser usado no Excell)  
