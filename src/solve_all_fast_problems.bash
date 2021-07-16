problem=$1
while [ $problem -le $2 ]
do
	#if test -f ../solutions/$problem.solution; then
		echo solving $problem
		timeout 30s ./main $problem >> ../logs/$problem.log
		#echo $problem solved
	#fi
	((problem++))
done
echo ""
echo All done