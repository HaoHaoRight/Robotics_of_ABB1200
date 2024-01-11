# Rebuild

1. 不使用元胞数组
2. 距离规定为norm(p1-p2)
3. 变量边为结构体，顶点为普通数组
4. 向量为行向量
5. 存入向量使用(i,:)

## TODO

1. 搜索半径r
2. 检查隐藏bug
3. 优化速度

## Algorithm 1:BIT*

1. V <- {x_strat}; E <- {}; X_sample <- {x_goal};

2. Q_E <- {};Q_V <- {};

3. **repeat**

   4. **if** Q_E == {} **and** if Q_E == {} **then**

      %  *Batch creation*

      5. **Prune**(gT(x_goal))
      6. X_samples <- **Sample**(m,gT(x_goal))
      7. V_old <- V;
      8. Q_V <- V;

   9. **while** BestValue(Q_V) <= BestValue(Q_E)

      %  *Vertex Expansion*

      10. **ExpandVertex**()

   %  *Edge Expansion*

   11. **ExpandEdge**()

12. **until** STOP
13. **return** T