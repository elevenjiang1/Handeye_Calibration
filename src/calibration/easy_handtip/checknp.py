import numpy as np

a=[np.array([[1,2,3],[4,5,6],[7,8,9]]),np.array([[3,2,1],[14,5,16],[27,84,9]])+3.5,np.array([[51,25,3],[43,5,36],[72,81,9]])+6.5]
b=[np.array([1,2,3]),np.array([4,5,6]),np.array([7,8,9])]
print("A:",a)


L=np.array(np.zeros((3,3)))
R=np.array(np.zeros((3,1)))
for i in range(len(a)):
    L=L+np.dot(a[i],a[i])
    R=R+np.dot(a[i],b[i])
    # print("L:",L)
    # print("a[i]:",a[i])
    # L=L+np.dot(a[i],a[i])
    # print("Now L:")
    # print(L)
print(L)
print("result:")
print(np.linalg.inv(L).dot(R))

result=np.linalg.inv(L).dot(R)



A=np.vstack(a)
B=np.vstack(b)
print("A:",A)
print("B:",B)
print("A.dot(result):(LR)")
print(A.dot(result))


result=np.linalg.inv(A.T.dot(A)).dot(A.T).dot(B)
print("My result:",result)
print("A.dot(result)")
print(A.dot(result))
