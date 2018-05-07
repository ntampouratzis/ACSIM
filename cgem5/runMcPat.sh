# This script executes the McPAT after GEM5 execution (it is executed automatically)
# It take 2 exactly parameters (e.g.: ./runMcPat.sh NodeNum XMLFile) #
#!/bin/sh
#check if gem5 and McPat files exists#
FILE_stats=$GEM5/node$1/stats.txt
FILE_config=$GEM5/node$1/config.json
FILE_xml=$GEM5/McPat/mcpat/ProcessorDescriptionFiles/$2
a=0
while [ $a -lt 5 ]
do
    sleep 1
    
    if ! [ -f "$FILE_stats" ]; then
      flag=0
    fi
    
    if ! [ -f "$FILE_config" ]; then
      flag=0
    fi
    
    if ! [ -f "$FILE_xml" ]; then
      flag=0
    fi
    
    if [ $flag -eq "1" ]; then
      break
    fi
   
   a=`expr $a + 1`
done

if [ $flag -eq "0" ]; then
    
    if ! [ -f "$FILE_stats" ]; then
      echo "File $FILE_stats does not exist!"
    fi
    
    if ! [ -f "$FILE_config" ]; then
      echo "File $FILE_config does not exist!"
    fi
    
    if ! [ -f "$FILE_xml" ]; then
      echo "File $FILE_xml does not exist!"
    fi
fi
#END check if gem5 and McPat files exists#

$GEM5/McPat/Scripts/GEM5ToMcPAT.py $GEM5/node$1/stats.txt $GEM5/node$1/config.json $GEM5/McPat/mcpat/ProcessorDescriptionFiles/$2 -o $GEM5/McPat/mcpatNode$1.xml
$GEM5/McPat/mcpat/mcpat -infile $GEM5/McPat/mcpatNode$1.xml -print_level 1 > $GEM5/McPat/mcpatOutput$1.txt
$GEM5/McPat/Scripts/print_energy.py $GEM5/McPat/mcpatOutput$1.txt $GEM5/node$1/stats.txt > $GEM5/McPat/energy$1.txt


