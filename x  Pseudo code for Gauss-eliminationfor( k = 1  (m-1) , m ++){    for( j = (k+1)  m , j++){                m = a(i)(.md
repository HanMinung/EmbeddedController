```c
// Pseudo code of back substitution
Create Matrix : size with (_b.rows x 1)

for ( i = n ~ 1 , i--){
   for( i = (i+1) ~ n , i++){
       sum = sum + (b_i - a(i)(j) * X_i);
       X_i = (b_i - sum)/a(i)(i);
       
   }
   // get solution in reverse index order 
   b(i)(0) = X_i; 
}
```

