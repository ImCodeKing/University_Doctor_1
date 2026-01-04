from constraint import Problem, AllDifferentConstraint

problem = Problem()

houses = [1, 2, 3, 4, 5]

nations = ["UK", "Spain", "Norway", "Ukraine", "Japan"]
colors = ["Red", "Green", "Ivory", "Yellow", "Blue"]
candies = ["Hershey", "KitKat", "Smarties", "Snickers", "MilkyWay"]
drinks = ["Tea", "Milk", "OrangeJuice", "Water", "Drink5"]
pets = ["Dog", "Fox", "Snail", "Horse", "Zebra"]

for var in nations + colors + candies + drinks + pets:
    problem.addVariable(var, houses)

problem.addConstraint(AllDifferentConstraint(), nations)
problem.addConstraint(AllDifferentConstraint(), colors)
problem.addConstraint(AllDifferentConstraint(), candies)
problem.addConstraint(AllDifferentConstraint(), drinks)
problem.addConstraint(AllDifferentConstraint(), pets)

# 英国人住在红色房子里
problem.addConstraint(lambda uk, red: uk == red, ("UK", "Red"))
# 西班牙人养狗
problem.addConstraint(lambda sp, dog: sp == dog, ("Spain", "Dog"))
# 挪威人住在最左边
problem.addConstraint(lambda n: n == 1, ("Norway",))
# 绿房子在象牙色房子的右边
problem.addConstraint(lambda g, i: g == i + 1, ("Green", "Ivory"))
# Hershey 的人住在养狐狸的人旁边
problem.addConstraint(lambda h, f: abs(h - f) == 1, ("Hershey", "Fox"))
# 黄色房子的人喜欢 KitKat
problem.addConstraint(lambda y, k: y == k, ("Yellow", "KitKat"))
# 挪威人住在蓝色房子旁边
problem.addConstraint(lambda n, b: abs(n - b) == 1, ("Norway", "Blue"))
# Smarties → 蜗牛
problem.addConstraint(lambda s, sn: s == sn, ("Smarties", "Snail"))
# Snickers → 橘汁
problem.addConstraint(lambda s, o: s == o, ("Snickers", "OrangeJuice"))
# 乌克兰人喝茶
problem.addConstraint(lambda u, t: u == t, ("Ukraine", "Tea"))
# 日本人喜欢 MilkyWay
problem.addConstraint(lambda j, m: j == m, ("Japan", "MilkyWay"))
# KitKat 的人住在养马人的隔壁
problem.addConstraint(lambda k, h: abs(k - h) == 1, ("KitKat", "Horse"))
# 中间房子喝牛奶
problem.addConstraint(lambda m: m == 3, ("Milk",))

solutions = problem.getSolutions()

print(f"找到解的数量: {len(solutions)}")

solution = solutions[0]
for house in houses:
    print(f"\n房子 {house}:")
    for category in [nations, colors, candies, drinks, pets]:
        for item in category:
            if solution[item] == house:
                print(f"  {item}")
