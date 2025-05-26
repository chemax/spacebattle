package main

import (
	"fmt"
	"math"
	"math/rand"
)

const (
	G             = 9.81
	SimStep       = 0.01
	MaxTime       = 60
	MaxAccel      = 10 * G
	ShipRadius    = 30.0
	EffectiveDist = 30000.0
	Pellets       = 24          // дробинок за выстрел
	SpreadAngle   = math.Pi / 3 // веер картечи (30 градусов)
)

type Vec2 struct {
	X, Y float64
}

func (v Vec2) Add(v2 Vec2) Vec2     { return Vec2{v.X + v2.X, v.Y + v2.Y} }
func (v Vec2) Sub(v2 Vec2) Vec2     { return Vec2{v.X - v2.X, v.Y - v2.Y} }
func (v Vec2) Len() float64         { return math.Hypot(v.X, v.Y) }
func (v Vec2) Scale(s float64) Vec2 { return Vec2{v.X * s, v.Y * s} }
func (v Vec2) Norm() Vec2 {
	l := v.Len()
	if l == 0 {
		return Vec2{0, 0}
	}
	return v.Scale(1.0 / l)
}
func (v Vec2) Rotate(rad float64) Vec2 {
	return Vec2{v.X*math.Cos(rad) - v.Y*math.Sin(rad), v.X*math.Sin(rad) + v.Y*math.Cos(rad)}
}

type Gun struct {
	ROF         float64
	ProjectileV float64
	Damage      float64
	Reload      float64
}

type Projectile struct {
	Pos        Vec2
	Vel        Vec2
	Target     *Ship
	Damage     float64
	Alive      bool
	FlightTime float64
}

type Ship struct {
	Name    string
	Pos     Vec2
	Vel     Vec2
	Armor   float64
	Guns    []Gun
	Alive   bool
	Target  *Ship
	Evasion float64 // запас ускорения
}

func main() {
	ship1 := Ship{
		Name:    "Orion",
		Pos:     Vec2{0, 0},
		Vel:     Vec2{0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 0.5, ProjectileV: 3500, Damage: 40}, {ROF: 0.5, ProjectileV: 3500, Damage: 40}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship2 := Ship{
		Name:    "Valkyrie",
		Pos:     Vec2{10000, 0},
		Vel:     Vec2{0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 1, ProjectileV: 2500, Damage: 60}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship1.Target = &ship2
	ship2.Target = &ship1

	projectiles := []Projectile{}
	t := 0.0

	for ; t < MaxTime && ship1.Alive && ship2.Alive; t += SimStep {
		// 1. Маневрируем
		SmartManeuver(&ship1, projectiles)
		SmartManeuver(&ship2, projectiles)
		UpdatePhysics(&ship1)
		UpdatePhysics(&ship2)
		// 2. Оба стреляют картечью
		FireShotgun(&ship1, &ship2, &projectiles, Pellets, SpreadAngle)
		FireShotgun(&ship2, &ship1, &projectiles, Pellets, SpreadAngle)
		// 3. Апдейт снарядов + попадания
		projectiles = UpdateProjectiles(projectiles)
		if int(t*10)%100 == 0 {
			fmt.Printf("t=%.0fs: %s(%.1f) <-> %s(%.1f), dist=%.1f, активных пуль: %d\n",
				t, ship1.Name, ship1.Armor, ship2.Name, ship2.Armor, ship1.Pos.Sub(ship2.Pos).Len(), len(projectiles))
		}
	}
	fmt.Printf("\nФИНАЛ:\n")
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship1.Name, ship1.Armor, ship1.Alive)
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship2.Name, ship2.Armor, ship2.Alive)
}

// Умное маневрирование: если возможно, уходим с траектории хотя бы одного снаряда
func SmartManeuver(ship *Ship, projectiles []Projectile) {
	// Находим ближайшую угрозу по времени встречи
	nearest := -1.0
	evadeVec := Vec2{}
	for _, p := range projectiles {
		if !p.Alive || p.Target != ship {
			continue
		}
		relPos := p.Pos.Sub(ship.Pos)
		relVel := p.Vel.Sub(ship.Vel)
		dot := relPos.X*relVel.X + relPos.Y*relVel.Y
		if dot >= 0 { // снаряд уже улетел или летит мимо
			continue
		}
		tImpact := -dot / (relVel.X*relVel.X + relVel.Y*relVel.Y)
		if tImpact < 0 || tImpact > 5 { // уклоняемся только от близких угроз
			continue
		}
		// Сможем ли за это время "отползти" вбок?
		maxOffset := 0.5 * MaxAccel * tImpact * tImpact
		distToTraj := math.Abs(relPos.X*relVel.Y-relPos.Y*relVel.X) / relVel.Len()
		if maxOffset > distToTraj-ShipRadius {
			// Делаем уклонение "вбок" от снаряда (ортогонально его движению)
			ortho := Vec2{-relVel.Y, relVel.X}.Norm()
			if rand.Float64() < 0.5 {
				ortho = ortho.Scale(-1)
			}
			evade := ortho.Scale(MaxAccel * SimStep)
			// выбираем самое угрожающее направление
			if nearest < 0 || tImpact < nearest {
				evadeVec = evade
				nearest = tImpact
			}
		}
	}
	// Двигаемся: если нет угроз — двигаемся к цели
	if nearest < 0 {
		if ship.Target != nil && ship.Target.Alive {
			dir := ship.Target.Pos.Sub(ship.Pos).Norm()
			ship.Vel = ship.Vel.Add(dir.Scale(MaxAccel * SimStep))
		}
	} else {
		ship.Vel = ship.Vel.Add(evadeVec)
	}
}

// Физика движения
func UpdatePhysics(ship *Ship) {
	ship.Pos = ship.Pos.Add(ship.Vel.Scale(SimStep))
}

// Веерная стрельба
func FireShotgun(attacker, defender *Ship, projectiles *[]Projectile, nPellets int, spreadAngle float64) {
	if !attacker.Alive || !defender.Alive {
		return
	}
	dist := attacker.Pos.Sub(defender.Pos).Len()
	if dist > EffectiveDist {
		return
	}
	for i := range attacker.Guns {
		gun := &attacker.Guns[i]
		gun.Reload -= SimStep
		if gun.Reload <= 0 {
			gun.Reload = 1.0 / gun.ROF
			dir := defender.Pos.Sub(attacker.Pos).Norm()
			baseAngle := math.Atan2(dir.Y, dir.X)
			for n := 0; n < nPellets; n++ {
				angle := baseAngle + (rand.Float64()-0.5)*spreadAngle
				pelletDir := Vec2{math.Cos(angle), math.Sin(angle)}
				projVel := pelletDir.Scale(gun.ProjectileV).Add(attacker.Vel)
				*projectiles = append(*projectiles, Projectile{
					Pos:    attacker.Pos,
					Vel:    projVel,
					Target: defender,
					Damage: gun.Damage / float64(nPellets),
					Alive:  true,
				})
			}
		}
	}
}

// Снаряды: обновление и попадания
func UpdateProjectiles(projectiles []Projectile) []Projectile {
	newProj := projectiles[:0]
	for i := range projectiles {
		p := &projectiles[i]
		if !p.Alive || !p.Target.Alive {
			continue
		}
		p.Pos = p.Pos.Add(p.Vel.Scale(SimStep))
		p.FlightTime += SimStep
		// Попадание?
		if p.Pos.Sub(p.Target.Pos).Len() <= ShipRadius {
			ApplyDamage(p.Target, p.Damage)
			p.Alive = false
			continue
		}
		// Самоуничтожение снаряда если слишком долго летит (20 км)
		if p.FlightTime > EffectiveDist/p.Vel.Len()+1 {
			p.Alive = false
			continue
		}
		newProj = append(newProj, *p)
	}
	return newProj
}

func ApplyDamage(ship *Ship, dmg float64) {
	if ship.Armor > dmg {
		ship.Armor -= dmg
	} else {
		ship.Armor = 0
		ship.Alive = false
	}
}
