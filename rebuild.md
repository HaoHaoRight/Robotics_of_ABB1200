# Rebuild

1. 不使用元胞数组
2. 距离规定为norm(p1-p2)
3. 变量边为结构体，顶点为普通数组
4. 向量为行向量
5. 存入向量使用(i,:)

## Algorithm 1:BIT*

1. V <- {x_strat}; E <- {}; X_sample <- {x_goal};

2. Q_E <- {};Q_V <- {};

3. **repeat**

   4. **if** Q_E == {} **and** if Q_E == {} **then**

      %  *Batch creation*

      5. **Prune**(gT(x_goal))
      6. 