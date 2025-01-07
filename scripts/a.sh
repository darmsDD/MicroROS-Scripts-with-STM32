func_a() {
  echo "Function a"
  local new_array=(${@:3})
    # Print the new array
  echo "New array: ${new_array[@]}"
  echo ${new_array[7]}
  ("${a[@]:2}")
  for var in ${new_array[@]}
  do
      echo $var
  done
}

func_b() {
  local my_func=$1
  echo "The name of the function is: $my_func"
}

t(){
  echo "asdasd"
  kill 0
  echo "whyyyy"
}

meu_c(){
  echo "Passei no sigint"
  exit
}

print_phrases() {
    # Iterate over the arguments passed to the function
    for phrase in "$@"; do
        echo "$phrase"
    done
}

# Create an array with phrases
phrases=("This is the first phrase."
         "Here is another phrase."
         "And this is the last phrase.")

# Pass the array to the function
print_phrases "${phrases[@]}"


